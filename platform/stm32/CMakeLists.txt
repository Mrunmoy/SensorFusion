cmake_minimum_required(VERSION 3.16)

add_library(sensorfusion_platform_stm32 STATIC
    StmI2CBus.cpp
    StmSpiBus.cpp
    StmGpio.cpp
    StmAdcChannel.cpp
    StmDelay.cpp
    StmNvStore.cpp
)

target_include_directories(sensorfusion_platform_stm32 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(sensorfusion_platform_stm32 PUBLIC
    sensorfusion_drivers
)

target_compile_features(sensorfusion_platform_stm32 PUBLIC cxx_std_17)
