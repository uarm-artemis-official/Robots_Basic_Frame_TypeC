add_executable(MULTIBOARD_COMM_TEST multiboard_comm_test.cpp)
target_link_libraries(MULTIBOARD_COMM_TEST 
    PRIVATE 
        GTest::gtest_main
        TEST_TARGET 
        MULTIBOARD_FIXTURE)
gtest_add_tests(TARGET MULTIBOARD_COMM_TEST)

add_executable(SIMPLE_COMM_UTILS_REGRESSION simple_comm_utils_regression.cpp)
target_link_libraries(SIMPLE_COMM_UTILS_REGRESSION
    PRIVATE
        GTest::gtest_main
        TEST_TARGET
        MODULES::SIMPLE_COMM_UTILS)

gtest_add_tests(TARGET SIMPLE_COMM_UTILS_REGRESSION)