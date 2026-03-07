cmake_minimum_required(VERSION 3.16)
project(sensorfusion_esp32_example LANGUAGES CXX)

set(SENSORFUSION_PLATFORM "esp32" CACHE STRING "")
add_subdirectory(../../ SensorFusion)

add_executable(esp32_sensor_node main/main.cpp)
target_link_libraries(esp32_sensor_node PRIVATE
    sensorfusion_middleware
    sensorfusion_platform_esp32
)
