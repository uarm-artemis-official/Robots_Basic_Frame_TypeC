add_executable(FIRST_TEST first_test.cpp)
target_link_libraries(FIRST_TEST PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(FIRST_TEST)