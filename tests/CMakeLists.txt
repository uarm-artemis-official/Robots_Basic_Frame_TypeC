add_subdirectory(mocks)
add_subdirectory(fixtures)


add_executable(FIRST_TEST first_test.cpp)
target_link_libraries(FIRST_TEST PRIVATE GTest::gtest_main)

add_executable(PID_TEST pid_test.cpp)
target_link_libraries(PID_TEST 
    PRIVATE 
        GTest::gtest_main
        ALGORITHMS::CONTROL_LIB)

add_executable(UARM_MATH_TEST uarm_math_test.cpp)
target_link_libraries(UARM_MATH_TEST 
    PRIVATE 
        GTest::gtest_main
        LIB::UARM_MATH)

add_executable(RAMP_TEST ramp_test.cpp)
target_link_libraries(RAMP_TEST 
    PRIVATE
        GTest::gtest_main
        ALGORITHMS::CONTROL_LIB)

add_executable(CHASSIS_APP_TEST chassis_app_test.cpp)
target_include_directories(CHASSIS_APP_TEST 
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/fixtures)
target_link_libraries(CHASSIS_APP_TEST 
    PRIVATE
        GTest::gtest_main
        CHASSIS_APP_FIXTURE
        SUBSYSTEMS_MOCKS
        BASIC_APPS)



include(GoogleTest)
gtest_discover_tests(FIRST_TEST)
gtest_discover_tests(PID_TEST)
gtest_discover_tests(UARM_MATH_TEST)
gtest_discover_tests(RAMP_TEST)
gtest_discover_tests(CHASSIS_APP_TEST)