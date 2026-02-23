cmake_minimum_required(VERSION 3.16)

add_library(sensorfusion_middleware STATIC)

target_include_directories(sensorfusion_middleware PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/ahrs
    ${CMAKE_CURRENT_SOURCE_DIR}/hub
    ${CMAKE_CURRENT_SOURCE_DIR}/codec
    ${CMAKE_CURRENT_SOURCE_DIR}/altitude
    ${CMAKE_CURRENT_SOURCE_DIR}/ecg
)

target_compile_features(sensorfusion_middleware PUBLIC cxx_std_17)

target_link_libraries(sensorfusion_middleware PUBLIC sensorfusion_drivers)

target_sources(sensorfusion_middleware PRIVATE
    ahrs/MahonyAHRS.cpp
    hub/SensorHub.cpp
    codec/FrameCodec.cpp
    altitude/AltitudeEstimator.cpp
    ecg/HeartRateDetector.cpp
)
