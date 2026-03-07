cmake_minimum_required(VERSION 3.16)

add_library(sensorfusion_platform_esp32 STATIC
    EspI2CBus.cpp
    EspSpiBus.cpp
    EspGpio.cpp
    EspAdcChannel.cpp
    EspDelay.cpp
    EspNvStore.cpp
)

target_include_directories(sensorfusion_platform_esp32 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(sensorfusion_platform_esp32 PUBLIC
    sensorfusion_drivers
)

target_compile_features(sensorfusion_platform_esp32 PUBLIC cxx_std_17)
