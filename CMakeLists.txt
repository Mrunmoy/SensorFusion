cmake_minimum_required(VERSION 3.16)
project(SensorFusion VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SENSORFUSION_BUILD_TESTS "Build unit tests" ON)
option(SENSORFUSION_BUILD_APP "Build stub application" OFF)
set(SENSORFUSION_PLATFORM "none" CACHE STRING "Optional platform backend: none, esp32, stm32, nrf52")
set_property(CACHE SENSORFUSION_PLATFORM PROPERTY STRINGS none esp32 stm32 nrf52)

add_subdirectory(drivers)
add_subdirectory(middleware)

if(NOT SENSORFUSION_PLATFORM STREQUAL "none")
    add_subdirectory(platform)
endif()

if(SENSORFUSION_BUILD_TESTS)
    add_subdirectory(test)
endif()

if(SENSORFUSION_BUILD_APP)
    add_executable(sensorfusion main/main.cpp)
    target_link_libraries(sensorfusion PRIVATE sensorfusion_middleware)
endif()
