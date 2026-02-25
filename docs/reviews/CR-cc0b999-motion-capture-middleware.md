# Code Review: Motion Capture Middleware -- FK, ZUPT, Calibration, Math Extensions

**Commit:** cc0b999b845a21e791e3aa3ae55f31143fd25052
**Date:** 2026-02-25
**Files Reviewed:** 21

## Summary

This commit adds a motion capture middleware layer to the SensorFusion library: a Vec3 struct,
Quaternion extensions (inverse, rotateVector, toRotationMatrix, slerp, fromAxisAngle), a
T-pose SegmentCalibration, LinearAccelExtractor, ZuptDetector, ForwardKinematics engine,
PoseSnapshot, a POSE frame type for FrameCodec, and 78 unit tests (307 total, all passing).
The mathematical implementations are correct and the static-only allocation strategy is
maintained throughout. Two critical safety issues exist in ForwardKinematics: a null pointer
dereference reachable via a valid-looking API call, and an unbounded array access on joint().

---

## Critical Issues

### [CR-001] ForwardKinematics::configure() -- Null Pointer Dereference
- **File:** `middleware/motion/ForwardKinematics.cpp`
- **Line(s):** 6-13
- **Severity:** CRITICAL
- **Description:** `configure(nullptr, 0)` is explicitly tested and correctly handles the
  zero-count case (the loop body never executes). However, `configure(nullptr, count)` with
  any `count > 0` immediately dereferences `bones[i]` in the loop body, causing undefined
  behaviour. There is no null guard before the loop. A caller that builds a skeleton array on
  the stack and accidentally passes the wrong count, or a fuzzer-style caller, will crash.
  The test suite only exercises the `(nullptr, 0)` case, leaving this path entirely untested.

  ```cpp
  // Current -- UB when bones==nullptr && count>0
  bool ForwardKinematics::configure(const Bone* bones, size_t count) {
      if (count > MAX_BONES) return false;
      for (size_t i = 0; i < count; ++i) {
          if (bones[i].parentIdx != ROOT && bones[i].parentIdx >= i)  // dereferences bones
  ```

- **Recommendation:** Add an explicit null guard before the loop:
  ```cpp
  if (count > 0 && bones == nullptr) return false;
  ```
  Also add a test: `EXPECT_FALSE(fk.configure(nullptr, 1));`

---

### [CR-002] ForwardKinematics::joint() -- No Bounds Check, Out-of-Bounds UB
- **File:** `middleware/motion/ForwardKinematics.hpp` and `middleware/motion/ForwardKinematics.cpp`
- **Line(s):** hpp:31, cpp:43-45
- **Severity:** CRITICAL
- **Description:** `joint(size_t index)` returns a reference directly into `joints_[index]`
  with no bounds check. `joints_` has exactly `MAX_BONES = 16` elements. Passing any index
  >= 16 is undefined behaviour (out-of-bounds array access). Additionally, if `configure()` is
  called twice (reconfigure with a smaller skeleton), `count_` shrinks but `joints_[count_..15]`
  hold stale data from the previous solve. A caller iterating as `joint(fk.boneCount() - 1)` is
  safe, but there is no API contract preventing `joint(0)` on an unconfigured (zero-bone) engine,
  which would return uninitialized data. In an embedded context where there is no MMU and no
  exception handling, silent data corruption is the likely outcome rather than a crash.

  ```cpp
  const JointPose& joint(size_t index) const {
      return joints_[index];  // No bounds check
  }
  ```

- **Recommendation:** Add a bounds check. Since exceptions are not used in this library, a
  saturating/clamped return or an assert is appropriate:
  ```cpp
  const JointPose& joint(size_t index) const {
      if (index >= count_) index = (count_ > 0) ? count_ - 1 : 0;
      return joints_[index];
  }
  ```
  Alternatively, document the precondition explicitly and add a test that calls `joint()` with
  `index == boneCount() - 1` as the boundary. At minimum, `index` must be validated against
  `count_` (not against `MAX_BONES`) since joints beyond `count_` may be uninitialised.

---

## Major Issues

### [CR-003] Quaternion::inverse() -- Missing Unit-Quaternion Precondition
- **File:** `middleware/common/Quaternion.hpp`
- **Line(s):** 32-34
- **Severity:** MAJOR
- **Description:** `inverse()` is implemented as a direct alias for `conjugate()`:
  ```cpp
  Quaternion inverse() const { return conjugate(); }
  ```
  This is mathematically correct ONLY for unit-length quaternions, where `|q| = 1` and
  therefore `q^-1 = q*`. For a non-unit quaternion the true inverse is `q* / |q|^2`. The
  function has no precondition comment and no normalisation step.

  All current callers happen to pass unit quaternions (AHRS output in LinearAccelExtractor;
  captureReference() in SegmentCalibration typically receives AHRS output), but `inverse()` is
  a public API and future callers could pass non-unit inputs. If a user calls
  `captureReference()` with a manufactured non-unit quaternion (e.g. constructed directly as
  `{2.0f, 0.0f, 0.0f, 0.0f}`), the resulting segment orientation will be incorrect with no
  diagnostic.

- **Recommendation:** Add a precondition comment at minimum. If strict correctness under all
  inputs is desired, divide by norm-squared:
  ```cpp
  // Precondition: quaternion must be unit-length (|q| == 1).
  // For unit quaternions, inverse == conjugate.
  Quaternion inverse() const { return conjugate(); }
  ```
  Consider also normalising inside `captureReference()` as a defensive measure:
  ```cpp
  void captureReference(const Quaternion& currentOrientation) {
      qRef_ = currentOrientation;
      qRef_.normalize();  // ensure unit for correct inverse() downstream
      calibrated_ = true;
  }
  ```

---

### [CR-004] Unicode Characters in New Test Files -- Coding Standard Violation
- **File:** `test/test_quaternion_ext.cpp`, `test/test_forward_kinematics.cpp`
- **Line(s):** test_quaternion_ext.cpp: 9, 30, 42, 52, 62, 78, 114, 161; test_forward_kinematics.cpp: 54, 72, 84, 93, 129, 151
- **Severity:** MAJOR
- **Description:** The project coding standard explicitly states: "No emojis or unicode in
  code, comments, or markdown." Both new test files use Unicode box-drawing characters
  (U+2500 EN DASH `─`, U+2014 EM DASH `—`) and Unicode arrows (U+2192 `→`) in comments:

  ```
  // ── inverse ──────────────────────────────────────────────────────────────────
  // 90 deg about Z: +X → +Y
  // Elbow at (0, 0.3, 0) — uses parent orientation (identity) to place child
  ```

  This causes non-UTF-8-safe toolchains and grep/diff tools to emit warnings, and violates
  the stated standard. Note that the pre-existing `Quaternion.hpp` and `FrameCodec.hpp` also
  contain Unicode dashes, but those are pre-existing issues and not introduced by this commit.
  The new test files are the violations introduced here.

- **Recommendation:** Replace all Unicode characters in the two test files:
  - Section dividers `// ──` -> `// ---` or `// ===`
  - Arrows `→` -> `->`
  - EM dashes `—` -> ` --` or ` -`

---

### [CR-005] `static constexpr` at Namespace Scope in Headers -- Redundant Linkage Specifier
- **File:** `middleware/motion/ForwardKinematics.hpp`, `middleware/motion/PoseSnapshot.hpp`
- **Line(s):** ForwardKinematics.hpp:10, PoseSnapshot.hpp:10
- **Severity:** MAJOR
- **Description:** Both files declare a file-level constant with `static constexpr`:

  ```cpp
  // ForwardKinematics.hpp
  static constexpr size_t MAX_BONES = 16;

  // PoseSnapshot.hpp
  static constexpr size_t MAX_JOINTS = 16;
  ```

  In C++17, `constexpr` variables at namespace scope have external linkage by default unless
  they are also declared `inline`. Adding `static` gives them internal linkage, which means
  each translation unit that includes these headers gets its own copy of the symbol. This is
  a redundant qualifier that pollutes the global namespace and creates confusion. The project
  convention for class constants is to make them `static constexpr` class members (as is done
  correctly with `ForwardKinematics::ROOT` and `ZuptDetector::WINDOW_SIZE`). File-level
  constants that are only used inside their respective classes should be class-scope members
  instead.

- **Recommendation:** Move `MAX_BONES` into the `ForwardKinematics` class body as a public
  `static constexpr` member:
  ```cpp
  class ForwardKinematics {
  public:
      static constexpr size_t MAX_BONES = 16;
      static constexpr uint8_t ROOT = 0xFF;
      ...
  ```
  Similarly move `MAX_JOINTS` into a `PoseSnapshot` scope (or a named `namespace sf` constant
  without `static`). This matches the pattern used for `ROOT` and `WINDOW_SIZE`.

---

### [CR-006] MAX_BONES and MAX_JOINTS Are Separate Unlinked Constants
- **File:** `middleware/motion/ForwardKinematics.hpp` (line 10), `middleware/motion/PoseSnapshot.hpp` (line 10)
- **Severity:** MAJOR
- **Description:** `MAX_BONES = 16` and `MAX_JOINTS = 16` are distinct constants defined
  independently in two different headers. `PoseSnapshot::positions[MAX_JOINTS]` and
  `ForwardKinematics::joints_[MAX_BONES]` are intended to be the same dimensioned array for
  a full-body pose. If one constant is ever updated without updating the other -- a likely
  maintenance error -- `ForwardKinematics::solve()` will write more joints than `PoseSnapshot`
  can hold, or a copy loop will overrun. There is no static assertion linking them.

- **Recommendation:** Define a single canonical `MAX_JOINTS` constant (e.g. in a shared
  `PoseSnapshot.hpp` or a new `MotionConfig.hpp`) and have `ForwardKinematics` reference it:
  ```cpp
  // In ForwardKinematics.hpp:
  #include "PoseSnapshot.hpp"
  class ForwardKinematics {
  public:
      static constexpr size_t MAX_BONES = MAX_JOINTS;  // or directly use MAX_JOINTS
  ```
  At minimum add a static assertion to catch divergence:
  ```cpp
  static_assert(MAX_BONES == MAX_JOINTS,
                "FK and PoseSnapshot joint counts must match");
  ```

---

### [CR-007] No Thread-Safety Documentation on Any New Class
- **File:** All new `.hpp` files in `middleware/motion/`
- **Line(s):** ForwardKinematics.hpp, ZuptDetector.hpp, SegmentCalibration.hpp, LinearAccelExtractor.hpp
- **Severity:** MAJOR
- **Description:** The project coding standard (from `instructions.md` and `CLAUDE.md`)
  requires: "Document thread safety expectations in class comments." None of the four new
  classes include such documentation. This is particularly important for motion capture
  pipelines where AHRS updates and FK solves may run on different RTOS tasks:

  - `ForwardKinematics`: stateful, `setNodeOrientation()` and `solve()` are not atomic.
    Concurrent calls from multiple RTOS tasks would corrupt `nodeOrientations_` or `joints_`.
  - `ZuptDetector`: accumulates a sliding window; `processSample()` and `isStationary()` are
    not safe to call from different contexts.
  - `SegmentCalibration`: `captureReference()` mutates `qRef_` and `calibrated_`; concurrent
    reads from `segmentOrientation()` would be a data race.
  - `LinearAccelExtractor`: read-only after construction (stateless beyond `gravityMag_`);
    effectively safe for concurrent reads but this is not documented.

- **Recommendation:** Add a class-level comment to each header:
  ```cpp
  // Not thread-safe. All methods must be called from the same task/thread,
  // or access must be protected by an external lock.
  class ForwardKinematics { ...
  ```
  For LinearAccelExtractor, note that it is safe to call const methods concurrently once
  constructed.

---

## Minor Issues

### [CR-008] fromAxisAngle() -- Degrees Parameter Not Documented
- **File:** `middleware/common/Quaternion.hpp`
- **Line(s):** 96-104
- **Severity:** MINOR
- **Description:** `fromAxisAngle(float ax, float ay, float az, float angleDeg)` takes the
  angle in degrees, which is unusual in quaternion mathematics APIs (most use radians). The
  parameter is named `angleDeg` which hints at degrees, but there is no comment confirming
  this. A user coming from a radians-based API (e.g. GLSL, Eigen, STL) will produce silent
  errors. The `toEuler()` output is also in degrees, so there is consistency within the
  library, but it is undocumented.

- **Recommendation:** Add a brief comment:
  ```cpp
  // Constructs a unit quaternion from an axis-angle rotation.
  // angleDeg is in degrees. Axis (ax, ay, az) need not be pre-normalised.
  static Quaternion fromAxisAngle(float ax, float ay, float az, float angleDeg);
  ```

---

### [CR-009] LinearAccelExtractor -- Gravity Convention Undocumented
- **File:** `middleware/motion/LinearAccelExtractor.hpp` and `.cpp`
- **Line(s):** hpp:8-17, cpp:12, 27
- **Severity:** MINOR
- **Description:** The implementation assumes gravity in the world frame is `(0, 0, -g)`,
  which is the ENU (East-North-Up) or Z-up convention. This is opposite to NED
  (North-East-Down) convention where gravity is `(0, 0, +g)`. The class and its methods have
  no documentation describing the assumed coordinate frame. Users targeting NED platforms (e.g.
  aerospace applications) would get sign-flipped results with no indication of the issue.
  `AccelData` in `SensorTypes.hpp` is documented with units ("in g") but no frame.

- **Recommendation:** Add a class-level comment:
  ```cpp
  // Removes gravity from a raw accelerometer reading.
  //
  // Assumes ENU (Z-up) world coordinate frame:
  //   - Gravity in world frame: (0, 0, -gravityMagnitude)
  //   - A stationary sensor face-up reads approximately (0, 0, -1g).
  ```

---

### [CR-010] ZuptDetector -- Threshold Units Not Documented
- **File:** `middleware/motion/ZuptDetector.hpp`
- **Line(s):** 10, 18
- **Severity:** MINOR
- **Description:** `ZuptDetector` takes a `threshold` parameter (default `0.01f`) with no
  documentation of its units. Internally it computes the variance of `|accel|` (acceleration
  magnitude in g-units). The threshold is therefore in units of g^2 (squared g). A user who
  initialises with `0.01f` expecting threshold in g (not g^2) will get a detector tuned to a
  different sensitivity than intended. The `WINDOW_SIZE` of 32 samples is also an undocumented
  design constant (at 50 Hz this is 640 ms; at 200 Hz, 160 ms).

- **Recommendation:** Add a class-level comment:
  ```cpp
  // Detects zero-velocity periods by monitoring the variance of |accel| magnitude
  // over a sliding window of WINDOW_SIZE samples.
  //
  // threshold: maximum variance (in g^2) below which motion is considered absent.
  //   Default 0.01 g^2 suits 50-200 Hz IMUs. Increase for noisier sensors.
  // At 100 Hz, WINDOW_SIZE=32 gives a 320 ms detection window.
  ```

---

### [CR-011] FrameCodec::MAX_FRAME_SIZE Comment Mentions Only IMU_ALL
- **File:** `middleware/codec/FrameCodec.hpp`
- **Line(s):** 27
- **Severity:** MINOR
- **Description:** `MAX_FRAME_SIZE = 42` is commented as `// IMU_ALL: header(12) + 28 + crc(2)`.
  The new POSE frame type is also exactly 42 bytes (12 header + 28 payload + 2 CRC), so the
  constant value is still correct. However, the comment is now outdated since there are two
  frame types at the maximum size.

- **Recommendation:** Update the comment:
  ```cpp
  static constexpr size_t MAX_FRAME_SIZE = 42;  // IMU_ALL/POSE: header(12) + 28 + crc(2)
  ```

---

### [CR-012] Vec3 Missing Cross Product and Normalize Operations
- **File:** `middleware/common/Vec3.hpp`
- **Line(s):** 1-21
- **Severity:** MINOR
- **Description:** `Vec3` provides addition, subtraction, scalar multiplication, dot product,
  and length. It is missing `cross()` and `normalize()`. Neither is needed by the current
  implementations (the FK and LinearAccelExtractor use `Quaternion::rotateVector()` which
  handles the cross-product internals). However, `Vec3` is a public header included by users
  of the motion library and provides an incomplete API relative to what a 3D vector type is
  expected to provide. The omission of `normalize()` is particularly noticeable given that
  `Quaternion` has a `normalize()` method.

- **Recommendation:** Add the missing operations:
  ```cpp
  Vec3 cross(const Vec3& r) const {
      return {y * r.z - z * r.y, z * r.x - x * r.z, x * r.y - y * r.x};
  }

  Vec3 normalized() const {
      float len = length();
      if (len < 1e-10f) return {0.0f, 0.0f, 0.0f};
      return *this * (1.0f / len);
  }
  ```

---

### [CR-013] Bone Struct -- Padding and Alignment Not Documented
- **File:** `middleware/motion/ForwardKinematics.hpp`
- **Line(s):** 12-16
- **Severity:** MINOR
- **Description:** `struct Bone { uint8_t parentIdx; uint8_t nodeId; float length; }` has 2
  bytes of implicit padding between `nodeId` and `length` to align `length` to a 4-byte
  boundary. `sizeof(Bone)` is therefore 8 bytes (not 6), which means `Bone[16]` occupies 128
  bytes. This is not a bug, but on a flash-constrained MCU where a user might pass a `Bone`
  array from NV storage or over a serial link, the padding bytes carry undefined content and
  a direct memory copy will include garbage. The struct layout should be documented.

- **Recommendation:** Add a comment noting the layout, or use `__attribute__((packed))` if
  serialisation over wire or NV storage is intended:
  ```cpp
  // sizeof(Bone) = 8 bytes: 2 bytes fields + 2 bytes padding + 4 bytes float.
  // Not suitable for direct serialisation without accounting for padding.
  struct Bone {
      uint8_t parentIdx;
      uint8_t nodeId;
      float length;
  };
  ```

---

## Positive Observations

1. **Correct Rodrigues rotation formula.** `Quaternion::rotateVector()` uses the optimised
   cross-product form `v + 2w*(qv x v) + 2*(qv x (qv x v))` with the six cross-product
   multiplications inlined. The result is mathematically identical to the full sandwich
   product `q*p*q^-1` with 15 fewer multiplications. The test
   `RotationMatrixConsistentWithRotateVector` cross-validates `rotateVector()` against the
   independent matrix path, which is exactly the right strategy for verifying two
   implementations of the same operation.

2. **Slerp antipodal handling.** The `slerp()` implementation correctly detects the
   `dot < 0` case and negates `b` to take the shortest arc. The linear interpolation fallback
   for `dot > 0.9995` avoids the `acos(1.0)` / `sin(~0)` division-by-zero that most naive
   implementations miss. Both branches are covered by tests.

3. **ZuptDetector numerics.** The detector uses an online incremental sum/sum-of-squares
   accumulator rather than iterating the full window on every sample, giving O(1) per-sample
   cost. The variance is correctly clamped to 0 when floating-point arithmetic produces a
   small negative value. The test `VarianceComputedCorrectly` numerically validates the
   formula against a known input sequence.

4. **FK topological order enforcement.** `configure()` enforces that `parentIdx < i` for
   every non-root bone, guaranteeing that `solve()` processes bones in dependency order
   without requiring a separate topological sort. This is a clean O(n) invariant that also
   doubles as a structural validity check.

5. **Static-only allocation throughout.** Every new class uses fixed-size arrays with
   compile-time bounds. No `new`, `delete`, `std::vector`, or heap usage anywhere. All
   objects are suitable for stack or BSS placement on a cortex-M class device.

6. **Comprehensive test coverage.** All seven new components have dedicated test files. Edge
   cases tested include: zero-vector axis-angle, window-full behaviour in ZuptDetector,
   invalid topological order in FK, buffer-too-small in encodePose, and identity-quaternion
   passthrough in SegmentCalibration. The size tests (`sizeof` assertions) are a good
   practice for catching unexpected struct growth on target versus host.

7. **Float serialisation is correct.** `encodePose()` uses the same `writeFloat()` helper as
   all other frame encoders: `memcpy` to `uint32_t` then byte-by-byte little-endian write.
   This is standards-conforming (no type-punning UB) and endian-explicit. The round-trip test
   in `test_pose_codec.cpp` fully validates every payload field.

---

## Statistics

- Critical: 2
- Major: 5
- Minor: 6
- Files reviewed: 21
- Lines added: 1238
- Lines removed: 0
