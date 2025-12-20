add_executable(MULTIBOARD_COMM_TEST multiboard_comm_test.cpp)
target_link_libraries(MULTIBOARD_COMM_TEST 
    PRIVATE 
        GTest::gtest_main
        TEST_TARGET 
        MULTIBOARD_FIXTURE)

gtest_add_tests(TARGET MULTIBOARD_COMM_TEST)