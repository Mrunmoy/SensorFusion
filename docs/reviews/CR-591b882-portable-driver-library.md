# Code Review: Portable Sensor Driver Library (dev/portable-drivers branch)

**Commit(s):** 68584e3 through 591b882 (19 commits)
**Branch:** dev/portable-drivers
**Date:** 2026-02-23
**Files Reviewed:** 47 source files (drivers, HAL interfaces, mocks, tests, CMake)
**Test results:** 128/128 tests passing

---

## Summary

This branch introduces a complete portable sensor driver library for embedded systems. It
provides HAL interfaces, eight sensor drivers (MPU6050, QMC5883L, BMP180, ADXL345, AD8232,
BMM350, LPS22DF, LSM6DSO), a CalibrationStore with CRC-32 integrity, and a factory test
framework. The architecture is clean and consistently applied across all drivers. All 128 tests
pass. The code is well-structured and the byte-order handling, interrupt patterns, and error
propagation are handled consistently. Several correctness issues exist that require attention
before targeting production hardware.

---

## Critical Issues

### CR-001 Implicit narrowing conversion in sensorToHost16 shifts (all drivers)
- **Files:**
  - `drivers/mpu6050/MPU6050.cpp` line 38
  - `drivers/bmp180/BMP180.cpp` lines 39, 42, 65
  - `drivers/qmc5883l/QMC5883L.cpp` line 26
  - `drivers/adxl345/ADXL345.cpp` line 24
  - `drivers/bmm350/BMM350.cpp` line 46
  - `drivers/lps22df/LPS22DF.cpp` line 25
  - `drivers/lsm6dso/LSM6DSO.cpp` line 51
- **Severity:** CRITICAL
- **Description:** Every `sensorToHost16` function performs `(buf[N] << 8)` where `buf[N]` is
  `uint8_t`. On any platform where `int` is 16 bits (common for AVR, MSP430, and some 8/16-bit
  targets this library may target), shifting a `uint8_t` value of 0x80 or higher left by 8
  bits causes signed integer overflow -- undefined behavior. Even on 32-bit platforms the result
  is a signed `int`, so when `buf[0]` is >= 0x80 the shift produces a large positive int that
  is then truncated to `int16_t`, yielding the correct two's-complement result by accident. The
  BMP180 calibration lambdas (`calib[idx*2] << 8`) have the same issue.
- **Recommendation:** Cast to `uint16_t` before shifting to ensure unsigned arithmetic:
  ```cpp
  // Correct pattern:
  static int16_t sensorToHost16(const uint8_t* buf) {
      return static_cast<int16_t>(
          (static_cast<uint16_t>(buf[0]) << 8) | static_cast<uint16_t>(buf[1]));
  }
  // For little-endian variant:
  return static_cast<int16_t>(
      (static_cast<uint16_t>(buf[1]) << 8) | static_cast<uint16_t>(buf[0]));
  ```
  Apply the same pattern to all `(calib[idx*2] << 8)` expressions in `BMP180.cpp`. Since this
  library explicitly targets small MCUs, assume `int` can be 16 bits and write defensively.

---

### CR-002 ADXL345 scale factor is wrong when fullRes = false
- **File:** `drivers/adxl345/ADXL345.cpp` line 17, 53-55
- **Severity:** CRITICAL
- **Description:** `MG_PER_LSB = 0.004f` is applied unconditionally in `readAccel()`, but this
  value is only correct when `fullRes = true` (4 mg/LSB, independent of range). When
  `cfg_.fullRes = false`, the scale depends on the configured range:
  - +/-2g: 3.9 mg/LSB
  - +/-4g: 7.8 mg/LSB
  - +/-8g: 15.6 mg/LSB
  - +/-16g: 31.2 mg/LSB
  A user who constructs `ADXL345Config{.range = AdxlRange::G16, .fullRes = false}` will receive
  accelerometer readings that are 8x too small. The default config uses `fullRes = true` and
  `range = G4`, so the bug is not exposed by current tests.
- **Recommendation:** Either (a) refuse to construct with `fullRes = false` by ignoring the
  range (document that fullRes=false is unsupported), or (b) compute the correct scale at
  construction time using a lookup table for the non-full-res case, similar to how MPU6050 and
  LSM6DSO compute their scales:
  ```cpp
  static float lsbPerG(AdxlRange r, bool fullRes) {
      if (fullRes) return 256.0f;   // 3.9 mg/LSB = 1/256 g/LSB
      constexpr float table[] = {256.0f, 128.0f, 64.0f, 32.0f};
      return table[static_cast<uint8_t>(r)];
  }
  ```
  Store `lsbPerG_` and divide rather than multiply.

---

### CR-003 BMM350 duplicate OTP address constant hides a data alignment bug
- **File:** `drivers/bmm350/BMM350.cpp` lines 28-29
- **Severity:** CRITICAL
- **Description:** Two constants are defined with the same value:
  ```cpp
  static constexpr uint8_t OTP_WORD_OFF_Z  = 0x10;
  static constexpr uint8_t OTP_WORD_SENS_X = 0x10;  // dead, never used
  ```
  `readOtp()` reads word `0x10` once (via `OTP_WORD_OFF_Z`) and extracts both `offsetZ` from
  the high byte and `sensX` from the low byte. The intent is that the BMM350 packs two
  compensation values into a single OTP word. However, `OTP_WORD_SENS_X` is a dead constant
  that is never referenced in `readOtp()`, while the comment implies it should be a distinct
  address. If the packing is intentional (correct), the dead constant should be removed. If
  this reflects a misunderstanding of the BMM350 OTP map (where `sensX` occupies a different
  word from `offsetZ`), then `readOtp()` is reading wrong data for either `offsetZ` or `sensX`.
  Verify against the BMM350 datasheet (section on OTP word assignments) and either (a) remove
  `OTP_WORD_SENS_X` and document the dual-use of word `0x10`, or (b) fix the address.
- **Recommendation:** Cross-check the BMM350 datasheet OTP map. If the packing is confirmed
  correct, delete `OTP_WORD_SENS_X = 0x10` and document the word structure in a comment. If
  the addresses are wrong, fix them and add separate reads.

---

### CR-004 BMP180 computeTruePressure: division by zero if b4 == 0
- **File:** `drivers/bmp180/BMP180.cpp` lines 104-107
- **Severity:** CRITICAL
- **Description:** `computeTruePressure()` performs `(b7 * 2) / b4` and `(b7 / b4) * 2`
  where `b4` is a `uint32_t`. If `b4` equals zero (possible when `ac4_` is zero due to
  corrupted or uninitialized calibration data, or when `x3 + 32768` underflows to zero),
  this is integer division by zero, which is undefined behavior. On most embedded targets
  this raises a hardware fault (Cortex-M: UsageFault with DIVBYZERO if enabled). The
  datasheet does not guarantee `ac4_` is non-zero for all devices.
- **Recommendation:** Guard against `b4 == 0` before division. Since `b4 == 0` indicates
  bad calibration data that would produce meaningless pressure anyway, returning `false`
  is appropriate:
  ```cpp
  if (b4 == 0) return 0; // caller should treat result as invalid
  ```
  Alternatively add the guard at the call site in `computeTruePressure` by returning an
  error code, but that requires changing the signature. The safest embedded approach is
  an assertion or early return with a sentinel value, with a comment explaining why.

---

## Major Issues

### CR-005 CalibrationStore slotAddress: size_t to uint32_t narrowing
- **File:** `drivers/calibration/CalibrationStore.cpp` line 11
- **Severity:** MAJOR
- **Description:** `slotAddress()` returns `uint32_t`, but the multiplication
  `static_cast<uint32_t>(id) * SLOT_SIZE` has type `size_t` on 64-bit hosts because
  `SLOT_SIZE` is `constexpr size_t`. On a 64-bit build host the multiplication is 64-bit
  and is then implicitly narrowed to `uint32_t` for the return. The compiler warning
  `-Wconversion` flags this exact line. On embedded 32-bit targets `size_t` is 32-bit so
  there is no narrowing, but the host build (used for tests) silently truncates. If the
  library is later used with a larger `SLOT_SIZE` and more sensor IDs, this could corrupt
  NV store accesses on 64-bit test hosts.
- **Recommendation:** Cast explicitly to make the narrowing intentional and suppress the
  warning honestly:
  ```cpp
  return baseAddr_ + static_cast<uint32_t>(
      static_cast<uint32_t>(id) * static_cast<uint32_t>(SLOT_SIZE));
  ```
  Or change `SLOT_SIZE` to `uint32_t`:
  ```cpp
  static constexpr uint32_t SLOT_SIZE = 4u + sizeof(CalibrationData) + 4u;
  ```

---

### CR-006 reg:: namespace is named, not anonymous -- potential future ODR hazard
- **Files:** All 7 driver `.cpp` files (MPU6050.cpp, QMC5883L.cpp, BMP180.cpp, ADXL345.cpp,
  BMM350.cpp, LPS22DF.cpp, LSM6DSO.cpp)
- **Severity:** MAJOR
- **Description:** Every driver defines `namespace sf { namespace reg { ... } }` with `constexpr`
  register address constants. In C++17 `constexpr` variables at namespace scope have internal
  linkage, so there is no ODR violation and no current link-time issue. However, the use of the
  named namespace `sf::reg` is fragile: if any driver header is ever included in another
  translation unit that also includes a second driver header, name collisions like
  `sf::reg::WHO_AM_I` (defined with value 0x75 in MPU6050.hpp context, 0x0F in LSM6DSO context)
  would cause hard-to-debug issues. The conventional pattern for file-scoped constants that
  should not be visible outside the TU is an anonymous namespace.
- **Recommendation:** Change all driver `.cpp` files to use an anonymous namespace or, as a
  middle ground, a driver-specific named namespace:
  ```cpp
  // Instead of:
  namespace sf { namespace reg { ... } }
  // Use:
  namespace { /* or namespace sf::mpu6050_reg { ... } */ }
  ```
  The anonymous namespace approach is idiomatic C++ for TU-local constants and eliminates
  any future collision risk.

---

### CR-007 ISPIBus lacks chip-select abstraction -- unusable for multi-device SPI buses
- **File:** `drivers/hal/ISPIBus.hpp`
- **Severity:** MAJOR
- **Description:** The `ISPIBus` interface has no chip-select mechanism. SPI buses are inherently
  shared between multiple devices, each with a dedicated CS line. Without CS control in the
  interface (or at minimum a way to assert/deassert CS per transaction), implementing a bus
  instance that manages multiple SPI sensors is impossible. Every caller must assume the
  `ISPIBus` instance is dedicated to a single device, which is an unusual hardware topology.
  The design doc (`hal_design.md`) does not mention this limitation.
- **Recommendation:** Add a chip-select pin identifier to the interface or to the read/write
  methods. One common pattern for embedded SPI HALs is to include the CS GPIO pin as a
  constructor parameter for a device-specific `ISPIBus` wrapper, making each wrapper
  instance device-specific. The current design implicitly assumes this pattern but does not
  document it. At minimum, add a comment to `ISPIBus.hpp`:
  ```cpp
  // Note: Each ISPIBus instance is assumed to represent one device (CS already asserted
  // by the implementation). For multi-device buses, create one ISPIBus instance per
  // device, with each implementation managing its own CS line.
  ```

---

### CR-008 BMM350 temperature register address is a magic number
- **File:** `drivers/bmm350/BMM350.cpp` line 198
- **Severity:** MAJOR
- **Description:** `readTemperature()` uses the literal `0x3A` as the temperature data register
  address. All other register addresses in BMM350.cpp are defined as named constants in the
  `reg::` namespace. This is inconsistent and makes the code harder to audit against the
  datasheet.
  ```cpp
  if (!bus_.readRegister(cfg_.address, 0x3A, buf, 3)) return false;
  ```
- **Recommendation:** Add `constexpr uint8_t TEMP_XLSB = 0x3A;` to the `reg::` namespace
  block and use it in `readTemperature()`.

---

### CR-009 FactoryTestRunner::runAll(results, maxResults) does not guard against null results
- **File:** `drivers/factory_test/FactoryTest.cpp` lines 16-22
- **Severity:** MAJOR
- **Description:** `runAll(TestResult* results, size_t maxResults)` dereferences `results[ran]`
  without checking if `results` is null. If a caller passes `nullptr` with `maxResults > 0`,
  this is a null pointer dereference. While defensive null checking is not always required in
  embedded C++, the API signature does not constrain callers and the failure mode (hard fault)
  is silent.
- **Recommendation:** Either (a) add an explicit null guard at the top of the function:
  ```cpp
  if (!results) return 0;
  ```
  or (b) document with a comment that `results` must be non-null when `maxResults > 0`, and
  add an assertion:
  ```cpp
  // Precondition: if maxResults > 0, results must point to at least maxResults elements.
  ```

---

### CR-010 CalibrationData float serialization is not cross-platform safe
- **File:** `drivers/calibration/CalibrationStore.cpp` lines 36, 61
- **Severity:** MAJOR
- **Description:** `save()` uses `std::memcpy(&data, ...)` to serialize `CalibrationData`
  (which contains `float` members) directly into a byte buffer for NV storage. `float` is
  stored in IEEE 754 little-endian format on ESP32, nRF52, and STM32, which are the stated
  targets. However, no static_assert confirms this, and the design doc does not document
  the byte-order assumption. If the library is ported to a big-endian target (e.g., some
  Power or MIPS MCUs), NV data written on one target would be silently misinterpreted on
  another.
- **Recommendation:** Add a `static_assert` to guard the assumption:
  ```cpp
  static_assert(__FLOAT_WORD_ORDER__ == __ORDER_LITTLE_ENDIAN__,
      "CalibrationStore NV format assumes little-endian float storage");
  ```
  Document in the class header that the NV format is platform-specific and non-portable
  across endianness boundaries.

---

### CR-011 Missing test coverage for interrupt enable bus failure in LSM6DSO, LPS22DF, BMM350
- **Files:**
  - `test/test_lsm6dso.cpp`
  - `test/test_lps22df.cpp`
  - `test/test_bmm350.cpp`
- **Severity:** MAJOR
- **Description:** `test_mpu6050.cpp`, `test_adxl345.cpp`, and `test_qmc5883l.cpp` each have
  an `EnableInterruptBusFail` test that verifies the bus write failure path in
  `enableDataReadyInterrupt()`. The three newer drivers (LSM6DSO, LPS22DF, BMM350) are
  missing this test case entirely. The error path exists in the driver code but is untested.
- **Recommendation:** Add `EnableInterruptBusFail` tests to each of the three test files,
  following the same pattern as `ADXL345Test::EnableInterruptBusFail`:
  ```cpp
  TEST_F(LSM6DSOTest, EnableInterruptBusFail) {
      expectFullInit();
      LSM6DSO imu(bus, delay);
      ASSERT_TRUE(imu.init());
      MockGpioInterrupt intPin;
      EXPECT_CALL(bus, writeRegister(ADDR, REG_INT1_CTRL, _, 1))
          .WillOnce(Return(false));
      EXPECT_FALSE(imu.enableDataReadyInterrupt(&intPin, nullptr, nullptr));
  }
  ```

---

## Minor Issues

### CR-012 ADXL345 sensitivity comment is slightly inaccurate (3.9 vs 4 mg/LSB)
- **File:** `drivers/adxl345/ADXL345.cpp` line 17
- **Severity:** MINOR
- **Description:** The comment says "4 mg/LSB in full-resolution mode" but the ADXL345
  datasheet specifies 3.9 mg/LSB (typical). Using 0.004f introduces approximately 2.5%
  scale error across all readings in full-resolution mode.
- **Recommendation:** Change to `0.0039f` and update the comment to "3.9 mg/LSB (typical,
  per ADXL345 datasheet Table 1)".

---

### CR-013 QMC5883L enableDataReadyInterrupt clobbers CTRL2 without read-modify-write
- **File:** `drivers/qmc5883l/QMC5883L.cpp` lines 87, 100
- **Severity:** MINOR
- **Description:** `enableDataReadyInterrupt()` writes `0x01` to CTRL2, and
  `disableDataReadyInterrupt()` writes `0x00`. The QMC5883L CTRL2 register also contains
  the SOFT_RST bit (0x80) and ROL_PNT (compass point rolling, 0x40). Writing these values
  without reading the current register state means any previously-set CTRL2 bits (other than
  INT_ENB) are silently cleared. In typical sensor init sequences CTRL2 is only written
  during init (soft reset to 0x80, then left cleared), so in practice this does not matter.
  However, if `ROL_PNT` or other CTRL2 bits are ever used, this becomes a latent bug.
- **Recommendation:** Document the assumption that CTRL2[7:1] are zero when
  `enableDataReadyInterrupt` is called, or perform a read-modify-write:
  ```cpp
  uint8_t ctrl2 = 0;
  if (!bus_.read8(addr, reg::CTRL2, ctrl2)) return false;
  ctrl2 |= 0x01; // set INT_ENB
  if (!bus_.write8(addr, reg::CTRL2, ctrl2)) return false;
  ```

---

### CR-014 BMM350 dead constant OTP_WORD_SENS_X (tied to CR-003)
- **File:** `drivers/bmm350/BMM350.cpp` line 29
- **Severity:** MINOR
- **Description:** `OTP_WORD_SENS_X = 0x10` is defined but never referenced in `readOtp()`.
  This is both dead code and potentially confusing since it shares the same value as
  `OTP_WORD_OFF_Z = 0x10`. Even if the OTP packing is intentional, the unused constant
  misleads readers into thinking `sensX` has its own OTP word address.
- **Recommendation:** Remove `OTP_WORD_SENS_X` and add a comment to the `OTP_WORD_OFF_Z`
  read explaining that this word packs both `offsetZ` (high byte) and `sensX` (low byte).

---

### CR-015 Inconsistent test expectation placement in QMC5883L tests
- **File:** `test/test_qmc5883l.cpp` lines 51-79
- **Severity:** MINOR
- **Description:** In `IsDataReady` and `IsDataNotReady` tests, the `EXPECT_CALL` for the
  STATUS register read is placed before `expectInit()`. In all other test files, expectations
  are set up in the order they will be called, or all expectations are set up before
  constructing the device. The reversed ordering is valid because GMock resolves calls against
  all registered expectations, but the style is inconsistent with the rest of the test suite
  and can confuse reviewers about call ordering.
- **Recommendation:** Move the STATUS register `EXPECT_CALL` to after `expectInit()` in both
  tests, placing expectations in the order they will be satisfied:
  ```cpp
  TEST_F(QMC5883LTest, IsDataReady) {
      expectInit();
      QMC5883L mag(bus, delay);
      ASSERT_TRUE(mag.init());
      EXPECT_CALL(bus, readRegister(ADDR, REG_STATUS, _, 1)) ...;
      bool ready = false;
      ASSERT_TRUE(mag.isDataReady(ready));
      EXPECT_TRUE(ready);
  }
  ```

---

### CR-016 FactoryTestRunner pass/fail counters do not account for SKIPPED status
- **File:** `drivers/factory_test/FactoryTest.cpp` lines 18-22, 34-36
- **Severity:** MINOR
- **Description:** `runAll()` increments `passCount_` for `PASS` and `failCount_` for `FAIL`,
  but `SKIPPED` tests are silently not counted in either. There is no `skipCount()` accessor.
  A user monitoring factory test results via the counters has no way to know how many tests
  were skipped. The `runAll()` return value (number of tests run) includes SKIPPED tests,
  which is also inconsistent -- SKIPPED tests arguably were not "run" in any meaningful sense.
- **Recommendation:** Add a `skippedCount_` member and `skippedCount()` accessor, incrementing
  it when `status == TestStatus::SKIPPED`.

---

### CR-017 No SensorId validation in CalibrationStore::slotAddress
- **File:** `drivers/calibration/CalibrationStore.cpp` line 10-12
- **Severity:** MINOR
- **Description:** `slotAddress()` takes a `SensorId` parameter and multiplies its integer
  value by `SLOT_SIZE` without bounds checking. If an invalid `SensorId` value is cast from
  an integer (e.g., `static_cast<SensorId>(99)`), the computed address will be far out of
  bounds for the NV store, and the subsequent `nv_.read()` or `nv_.write()` call will fail
  (assuming the `INvStore` implementation bounds-checks). However, it would be cleaner to
  validate upfront. The enum has values 0, 1, 2; three is the implicit limit.
- **Recommendation:** Add a bounds check or a `static_assert` on the enum count. At minimum,
  document that valid sensor IDs are 0-2 and that the NV store must be at least
  `3 * SLOT_SIZE` bytes in capacity.

---

### CR-018 LSM6DSO readAll result layout comment is misleading
- **File:** `drivers/lsm6dso/LSM6DSO.cpp` lines 110-128
- **Severity:** MINOR
- **Description:** The comment at line 111 says "Read 14 bytes: temp(2) + gyro(6) + accel(6)
  from OUT_TEMP_L (0x20)". This is correct, but the gyro/accel offset comments within the
  burst read section reference "bytes 2-7" for gyro and "bytes 8-13" for accel. These offsets
  are zero-indexed within the buffer and are accurate. However, the layout described here
  (temp first, then gyro, then accel) is opposite to what many other 6-axis IMU burst reads
  return (accel first), which could cause confusion for readers familiar with MPU6050 layout.
  The comment is accurate but should be more explicit about the register-level layout.
- **Recommendation:** Expand the comment to reference actual register addresses:
  ```cpp
  // Burst read from 0x20: OUT_TEMP(0x20-0x21), OUTX_L_G(0x22-0x27), OUTX_L_A(0x28-0x2D)
  ```

---

### CR-019 BMP180 computeB5 potential division by zero (x1 + md_ == 0)
- **File:** `drivers/bmp180/BMP180.cpp` lines 83-85
- **Severity:** MINOR
- **Description:** `computeB5()` divides by `(x1 + md_)`. With valid calibration data from a
  real BMP180 sensor, `md_` is thousands (positive) and `x1` cannot be sufficiently negative
  to cause the sum to reach zero. However, if calibration data is zero-initialized (sensor not
  initialized, or `init()` not called), `md_` is zero and `x1` may be zero, causing integer
  division by zero. This is a programming error rather than a hardware error, but in safety-
  oriented embedded code it should be guarded.
- **Recommendation:** Since `readTemperature()` and `readPressure()` call `computeB5()` without
  verifying that `init()` succeeded, add a guard or an `initialized_` flag. At minimum,
  document that calling `read*()` without a prior successful `init()` produces undefined
  behavior.

---

### CR-020 MagData comment uses Unicode micro symbol in SensorTypes.hpp
- **File:** `drivers/hal/SensorTypes.hpp` line 8
- **Severity:** MINOR
- **Description:** The comment `// in µT` uses the Unicode micro sign (U+00B5, µ) in a header
  file. This is a non-ASCII character in a source file. While most modern toolchains handle
  UTF-8 source files without issue, some embedded tool chains (particularly older IAR, Keil,
  or MPLAB versions) will fail to compile or produce warnings on non-ASCII characters in
  source. Per the project's own coding standards, no unicode is permitted in code or comments.
- **Recommendation:** Replace `µT` with `uT` (ASCII micro approximation):
  ```cpp
  struct MagData   { float x, y, z; };  // in uT
  ```

---

## Positive Observations

**Consistent driver architecture.** Every sensor driver follows exactly the same pattern:
constructor taking HAL references, `init()` returning bool, named register constants in a
local namespace, early-return error propagation, and `sensorToHost16`/`sensorToHost24`
helpers for byte-order conversion. This discipline makes the codebase easy to audit and extend.

**Correct byte-order handling.** The commit that removed endian-named HAL helpers
(`9dadbb8`) was the right call. Byte-order conversion now lives in each driver where the
sensor's native byte order is known, and the HAL API is endianness-neutral. The `sensorToHost16`
and `sensorToHost24` implementations correctly handle both big-endian (BMP180, MPU6050) and
little-endian (ADXL345, QMC5883L, LSM6DSO, LPS22DF, BMM350) sensors.

**BMM350 sign-extend for 21-bit data.** The `signExtend21()` function correctly masks to
21 bits and sign-extends by OR-ing `0xFFE00000`. This is a subtle operation that is often
done incorrectly (e.g., casting an unsigned right-shift result). The test verifies both
positive and negative raw values.

**CRC-32 implementation is correct.** The software CRC-32 in `CalibrationStore::crc32()`
uses the correct ITU-T V.42 polynomial (0xEDB88320 reflected), the correct initial value
(0xFFFFFFFF), and the correct final XOR (0xFFFFFFFF). The test vector `"123456789"` ->
`0xCBF43926` confirms this.

**MockNvStore backing store.** The `useBackingStore()` helper in `MockNvStore.hpp` provides
a transparent in-memory backing store for round-trip tests without requiring `EXPECT_CALL`
boilerplate for every individual read/write. This is a clean pattern that allows
`CalibrationStoreTest` to test actual serialization logic rather than just mock interactions.

**BMP180 datasheet calibration example.** Using the exact example values and expected
results from the BMP180 datasheet (AC1=408, UT=27898, T=15.0C, UP=23843, P=69964 Pa) as
test inputs validates the entire compensation formula chain end-to-end. This is significantly
more valuable than arbitrary test data.

**No dynamic allocation in driver code.** No driver or kernel-equivalent code uses `new`,
`delete`, `std::vector`, or any other heap-allocating container. All buffers are stack-local
arrays. The `MockNvStore` uses `std::vector` but this is correctly isolated to the test-only
mock, not production code.

**GPIO interrupt callback pattern is safe for ISR context.** The `Callback = void(*)(void*)`
function pointer type is appropriate for interrupt handlers on embedded targets -- no virtual
dispatch, no captures, no allocations. The `MockGpioInterrupt::captureCallback()` helper
correctly allows tests to simulate hardware interrupts synchronously.

**Factory test framework is minimal and sufficient.** The `FactoryTestRunner` with a fixed
`MAX_TESTS = 16` static array avoids dynamic allocation. The dual `runAll()` overloads
(one writing to a buffer, one invoking a callback) cover both batch and streaming use cases
for factory test output.

---

## Statistics

- **Critical:** 4
- **Major:** 7
- **Minor:** 9
- **Files reviewed:** 47
- **Lines added:** 5,231
- **Lines removed:** 630
