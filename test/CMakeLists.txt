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
    sensorfusion_drivers
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
    test_mpu6050.cpp
    test_qmc5883l.cpp
    test_bmp180.cpp
    test_adxl345.cpp
    test_ad8232.cpp
    test_bmm350.cpp
)

include(GoogleTest)
gtest_discover_tests(driver_tests)
