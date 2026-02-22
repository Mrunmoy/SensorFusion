cmake_minimum_required(VERSION 3.16)
project(SensorFusion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(drivers)
add_subdirectory(test)

add_executable(sensorfusion main/main.cpp)
target_link_libraries(sensorfusion PRIVATE sensorfusion_drivers)
