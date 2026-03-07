cmake_minimum_required(VERSION 3.16)
project(sensorfusion_nrf52_example LANGUAGES CXX)

set(SENSORFUSION_PLATFORM "nrf52" CACHE STRING "")
add_subdirectory(../../ SensorFusion)

add_executable(nrf52_motion_tracker main.cpp)
target_link_libraries(nrf52_motion_tracker PRIVATE
    sensorfusion_middleware
    sensorfusion_platform_nrf52
)
