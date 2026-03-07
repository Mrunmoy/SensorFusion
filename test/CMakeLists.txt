cmake_minimum_required(VERSION 3.16)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(driver_tests)

target_link_libraries(driver_tests PRIVATE
    sensorfusion_middleware
    GTest::gtest_main
    GTest::gmock
)

target_include_directories(driver_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
)

target_compile_features(driver_tests PRIVATE cxx_std_17)

target_sources(driver_tests PRIVATE
    placeholder_test.cpp
    test_hal.cpp
    test_at24cxx_nvstore.cpp
    test_mpu6050.cpp
    test_qmc5883l.cpp
    test_bmp180.cpp
    test_adxl345.cpp
    test_ad8232.cpp
    test_bmm350.cpp
    test_lps22df.cpp
    test_lsm6dso.cpp
    test_calibration.cpp
    test_calibration_at24cxx.cpp
    test_calibration_fitter.cpp
    test_factory.cpp
    test_quaternion.cpp
    test_ring_buffer.cpp
    test_mahony_ahrs.cpp
    test_sensor_hub.cpp
    test_frame_codec.cpp
    test_altitude_estimator.cpp
    test_heart_rate_detector.cpp
    test_vec3.cpp
    test_quaternion_ext.cpp
    test_segment_calibration.cpp
    test_linear_accel_extractor.cpp
    test_forward_kinematics.cpp
    test_zupt_detector.cpp
    test_pose_codec.cpp
    test_crc8_sensirion.cpp
    test_lis3mdl.cpp
    test_sht40.cpp
    test_sgp40.cpp
    test_bq25101.cpp
)

include(GoogleTest)
gtest_discover_tests(driver_tests)
