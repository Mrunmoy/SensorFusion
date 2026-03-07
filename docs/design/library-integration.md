# Library Integration & Distribution Design

## Problem

SensorFusion is a platform-agnostic embedded library. All drivers depend on
HAL interfaces (`II2CBus`, `IGpioInput`, `IDelayProvider`, etc.). A user on
ESP32, nRF52, or STM32 needs to:

1. Pull the library into their project
2. Get platform-specific HAL implementations (the "glue")
3. Instantiate drivers with the platform HAL and start using them

Without shipped platform backends, every user rewrites the same I2C/GPIO
wrappers. That's a bad experience and a source of bugs.

## Current Status (2026-03-07)

Phase 5 is now implemented in-repo:

- `platform/esp32`: ESP-IDF backend target `sensorfusion_platform_esp32`
- `platform/stm32`: STM32 HAL backend target `sensorfusion_platform_stm32`
- `platform/nrf52`: nRF52/nrfx backend target `sensorfusion_platform_nrf52`
- Example integration skeletons:
  - `examples/esp32-sensor-node/`
  - `examples/stm32-env-monitor/`
  - `examples/nrf52-motion-tracker/`

## Proposed Structure

```
SensorFusion/
├── drivers/                    # platform-agnostic sensor drivers
│   ├── hal/                    # HAL interfaces (II2CBus, IGpioInput, etc.)
│   ├── common/                 # shared utilities (CRC, etc.)
│   ├── mpu6050/                # sensor drivers...
│   └── ...
├── middleware/                  # AHRS, SensorHub, codecs, etc.
├── platform/                   # *** optional platform HAL backends ***
│   ├── esp32/
│   │   ├── EspI2CBus.hpp/cpp         # II2CBus → esp-idf i2c_master
│   │   ├── EspSpiBus.hpp/cpp         # ISPIBus → esp-idf spi_master
│   │   ├── EspGpio.hpp/cpp           # IGpioInput/Output/Interrupt → gpio
│   │   ├── EspAdcChannel.hpp/cpp     # IAdcChannel → adc_oneshot
│   │   ├── EspDelay.hpp/cpp          # IDelayProvider → vTaskDelay + esp_timer
│   │   ├── EspNvStore.hpp/cpp        # INvStore → NVS partition
│   │   └── CMakeLists.txt
│   ├── nrf52/
│   │   ├── NrfTwimBus.hpp/cpp        # II2CBus → nrfx_twim
│   │   ├── NrfSpimBus.hpp/cpp        # ISPIBus → nrfx_spim
│   │   ├── NrfGpio.hpp/cpp           # IGpio* → nrf_gpio + GPIOTE
│   │   ├── NrfSaadcChannel.hpp/cpp   # IAdcChannel → nrfx_saadc
│   │   ├── NrfDelay.hpp/cpp          # IDelayProvider → nrf_delay + RTC
│   │   ├── NrfFdsStore.hpp/cpp       # INvStore → FDS or littlefs
│   │   └── CMakeLists.txt
│   ├── stm32/
│   │   ├── StmI2CBus.hpp/cpp         # II2CBus → HAL_I2C
│   │   ├── StmSpiBus.hpp/cpp         # ISPIBus → HAL_SPI
│   │   ├── StmGpio.hpp/cpp           # IGpio* → HAL_GPIO
│   │   ├── StmAdcChannel.hpp/cpp     # IAdcChannel → HAL_ADC
│   │   ├── StmDelay.hpp/cpp          # IDelayProvider → HAL_Delay + DWT
│   │   ├── StmFlashStore.hpp/cpp     # INvStore → internal flash or EEPROM
│   │   └── CMakeLists.txt
│   └── README.md                # "Pick a platform or write your own"
├── examples/                    # *** working example projects ***
│   ├── esp32-sensor-node/       # complete ESP-IDF project
│   │   ├── main/
│   │   │   └── main.cpp         # wires platform HAL → drivers → middleware
│   │   ├── CMakeLists.txt
│   │   └── sdkconfig.defaults
│   ├── nrf52-motion-tracker/    # complete Zephyr or bare-metal project
│   └── stm32-env-monitor/      # complete STM32CubeIDE or CMake project
├── test/                        # unit tests (host-only, mock HAL)
├── CMakeLists.txt               # top-level: builds core lib + optional platform
└── README.md
```

## CMake Integration Model

### User's project (ESP-IDF example)

```cmake
# In user's project CMakeLists.txt:
add_subdirectory(components/SensorFusion)  # or FetchContent

target_link_libraries(my_app PRIVATE
    sensorfusion::drivers
    sensorfusion::middleware
    sensorfusion::platform_esp32   # optional — or write your own HAL
)
```

### Library's top-level CMakeLists.txt

```cmake
# Core targets (always built)
add_subdirectory(drivers)
add_subdirectory(middleware)

# Platform backend (opt-in)
set(SENSORFUSION_PLATFORM "none" CACHE STRING "Optional platform backend: none, esp32, stm32, nrf52")
if(NOT SENSORFUSION_PLATFORM STREQUAL "none")
    add_subdirectory(platform)
endif()

# Tests (host-only, off by default for embedded builds)
option(SENSORFUSION_BUILD_TESTS "Build unit tests" OFF)
if(SENSORFUSION_BUILD_TESTS)
    add_subdirectory(test)
endif()
```

## How a User Integrates

### Method 1: Git submodule (recommended)

```bash
cd my-esp32-project
git submodule add https://github.com/Mrunmoy/SensorFusion.git components/SensorFusion
```

```cmake
# CMakeLists.txt
set(SENSORFUSION_PLATFORM "esp32" CACHE STRING "")
add_subdirectory(components/SensorFusion)
```

### Method 2: CMake FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(sensorfusion
    GIT_REPOSITORY https://github.com/Mrunmoy/SensorFusion.git
    GIT_TAG        main
)
set(SENSORFUSION_PLATFORM "esp32" CACHE STRING "")
FetchContent_MakeAvailable(sensorfusion)
```

### Method 3: Custom HAL (no platform backend)

User implements the interfaces themselves:

```cpp
#include "II2CBus.hpp"

class MyI2CBus : public sf::II2CBus {
public:
    bool readRegister(uint8_t addr, uint8_t reg, uint8_t* buf, size_t len) override {
        // ... your platform I2C code ...
    }
    bool writeRegister(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) override {
        // ... your platform I2C code ...
    }
    bool probe(uint8_t addr) override { /* ... */ }
    bool rawWrite(uint8_t addr, const uint8_t* data, size_t len) override { /* ... */ }
    bool rawRead(uint8_t addr, uint8_t* buf, size_t len) override { /* ... */ }
};
```

Then wires it up:

```cpp
MyI2CBus i2c;
MyDelay  delay;

sf::MPU6050 imu(i2c, delay);
imu.init();

sf::AccelData accel;
imu.readAccel(accel);
```

## Example: Minimal ESP32 Sensor Node (main.cpp)

```cpp
#include "EspI2CBus.hpp"
#include "EspDelay.hpp"
#include "EspGpio.hpp"
#include "MPU6050.hpp"
#include "LIS3MDL.hpp"
#include "SHT40.hpp"
#include "SGP40.hpp"
#include "BQ25101.hpp"
#include "SensorHub.hpp"
#include "MahonyAHRS.hpp"

extern "C" void app_main() {
    // Platform HAL
    sf::EspI2CBus i2c(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
    sf::EspDelay  delay;
    sf::EspGpioInput  chgPin(GPIO_NUM_34);
    sf::EspGpioOutput tsPin(GPIO_NUM_25);

    // Drivers
    sf::MPU6050  imu(i2c, delay);
    sf::LIS3MDL  mag(i2c, delay);
    sf::SHT40    env(i2c, delay);
    sf::SGP40    voc(i2c, delay);
    sf::BQ25101  charger(chgPin, tsPin);

    imu.init();
    mag.init();
    env.init();
    voc.init();

    // Middleware
    sf::MahonyAHRS ahrs;
    sf::SensorHub  hub;
    hub.setIMU(&imu);
    hub.setMag(&mag);
    hub.setHumidity(&env);
    hub.setVoc(&voc);

    while (true) {
        sf::AccelData a; sf::GyroData g; sf::MagData m;
        if (imu.readAccel(a) && imu.readGyro(g) && mag.readMag(m)) {
            ahrs.update(a, g, m, 0.01f);
        }

        float humidity;
        uint16_t vocRaw;
        hub.readHumidity(humidity);
        hub.readVocRaw(vocRaw);

        if (charger.isCharging()) { /* show LED */ }

        delay.delayMs(10);
    }
}
```

## Implementation Priority

1. **Pick one platform first** (ESP32 recommended — most accessible, good tooling)
2. Implement the 6 HAL interfaces for that platform
3. Create one working example project end-to-end
4. Then replicate for nRF52 and STM32
5. Publish integration docs in README

## ESP-IDF Component Compatibility (future)

ESP-IDF has `idf_component_manager` with a component registry. To be
compatible, the library needs an `idf_component.yml` manifest at the root.
This is a future step after the CMake model works.

Similarly, Zephyr uses `west` modules with a `zephyr/module.yml`.
These are distribution channels built on top of the core CMake model.
