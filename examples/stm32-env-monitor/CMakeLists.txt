cmake_minimum_required(VERSION 3.16)
project(sensorfusion_stm32_example LANGUAGES CXX)

set(SENSORFUSION_PLATFORM "stm32" CACHE STRING "")
add_subdirectory(../../ SensorFusion)

add_executable(stm32_env_monitor main.cpp)
target_link_libraries(stm32_env_monitor PRIVATE
    sensorfusion_middleware
    sensorfusion_platform_stm32
)
