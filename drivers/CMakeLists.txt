cmake_minimum_required(VERSION 3.16)

add_library(sensorfusion_drivers STATIC)

target_include_directories(sensorfusion_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/mpu6050
    ${CMAKE_CURRENT_SOURCE_DIR}/qmc5883l
    ${CMAKE_CURRENT_SOURCE_DIR}/bmp180
    ${CMAKE_CURRENT_SOURCE_DIR}/adxl345
    ${CMAKE_CURRENT_SOURCE_DIR}/ad8232
)

target_compile_features(sensorfusion_drivers PUBLIC cxx_std_17)

target_sources(sensorfusion_drivers PRIVATE
    placeholder.cpp
    mpu6050/MPU6050.cpp
    qmc5883l/QMC5883L.cpp
    bmp180/BMP180.cpp
    adxl345/ADXL345.cpp
)
