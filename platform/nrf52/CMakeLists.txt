cmake_minimum_required(VERSION 3.16)

add_library(sensorfusion_platform_nrf52 STATIC
    NrfTwimBus.cpp
    NrfSpimBus.cpp
    NrfGpio.cpp
    NrfSaadcChannel.cpp
    NrfDelay.cpp
    NrfFdsStore.cpp
)

target_include_directories(sensorfusion_platform_nrf52 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(sensorfusion_platform_nrf52 PUBLIC
    sensorfusion_drivers
)

target_compile_features(sensorfusion_platform_nrf52 PUBLIC cxx_std_17)
