add_executable(FIRST_TEST first_test.cpp)
target_link_libraries(FIRST_TEST PRIVATE GTest::gtest_main)

add_executable(PID_TEST pid_test.cpp)
target_link_libraries(PID_TEST 
    PRIVATE 
        GTest::gtest_main
        ALGORITHMS::CONTROL_LIB)

add_executable(UARM_LIBRARIES_TEST maths_test.cpp)
target_link_libraries(UARM_LIBRARIES_TEST 
    PRIVATE 
        GTest::gtest_main
        LIB::UARM_MATH)


include(GoogleTest)
gtest_discover_tests(FIRST_TEST)
gtest_discover_tests(PID_TEST)
gtest_discover_tests(UARM_LIBRARIES_TEST)