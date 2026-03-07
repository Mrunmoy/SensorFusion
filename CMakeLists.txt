cmake_minimum_required(VERSION 3.16)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" SENSORFUSION_VERSION LIMIT_COUNT 1)
if(NOT SENSORFUSION_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
    message(FATAL_ERROR "Invalid VERSION file format: '${SENSORFUSION_VERSION}'")
endif()

project(SensorFusion VERSION ${SENSORFUSION_VERSION} LANGUAGES CXX)

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
