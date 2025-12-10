include(GoogleTest)

add_subdirectory(mocks)
add_subdirectory(fakes)
add_subdirectory(fixtures)

add_library(TEST_TARGET INTERFACE)
target_compile_options(TEST_TARGET INTERFACE --coverage)
target_link_options(TEST_TARGET INTERFACE --coverage)

add_subdirectory(regression)

add_executable(PID_TEST pid_test.cpp)
target_link_libraries(PID_TEST 
    PRIVATE 
        GTest::gtest_main
        TEST_TARGET 
        ALGORITHMS::CONTROL_LIB)

add_executable(UARM_MATH_TEST uarm_math_test.cpp)
target_link_libraries(UARM_MATH_TEST 
    PRIVATE 
        GTest::gtest_main
        TEST_TARGET 
        LIB::UARM_MATH)

add_executable(RAMP_TEST ramp_test.cpp)
target_link_libraries(RAMP_TEST 
    PRIVATE
        GTest::gtest_main
        TEST_TARGET 
        ALGORITHMS::CONTROL_LIB)

add_executable(MOCK_RTOS_QUEUE_TEST mock_rtos_queue_test.cpp)
target_link_libraries(MOCK_RTOS_QUEUE_TEST
    PRIVATE
        GTest::gtest_main
        TEST_TARGET 
        MW::MIDDLEWARE)

add_executable(MESSAGE_CENTER_TEST message_center_test.cpp)
target_link_libraries(MESSAGE_CENTER_TEST
    PRIVATE
        GTest::gtest_main
        TEST_TARGET
        MW::MIDDLEWARE
        MODULES::MESSAGE_CENTER)

add_executable(FAKE_MESSAGE_CENTER_TEST fake_message_center_test.cpp)
target_link_libraries(FAKE_MESSAGE_CENTER_TEST
    PRIVATE
        GTest::gtest_main
        TEST_TARGET
        MW::MIDDLEWARE
        MIDDLEWARE_MOCKS
        MODULES::MESSAGE_CENTER
        LIB::FAKE_MESSAGE_CENTER)

add_executable(COMMUNICATION_TEST communication_test.cpp)
target_link_libraries(COMMUNICATION_TEST
    PRIVATE
        GTest::gtest_main
        MIDDLEWARE_MOCKS
        SUBSYSTEMS::COMMUNICATION
        MODULES::MESSAGE_CENTER)

add_executable(FIFO_TEST fifo_test.cpp)
target_link_libraries(FIFO_TEST
    PRIVATE
        GTest::gtest_main
        TEST_TARGET
        ALGORITHMS::FIFO)


gtest_discover_tests(PID_TEST)
gtest_discover_tests(UARM_MATH_TEST)
gtest_discover_tests(RAMP_TEST)
gtest_discover_tests(MOCK_RTOS_QUEUE_TEST)
gtest_discover_tests(MESSAGE_CENTER_TEST)
gtest_discover_tests(FAKE_MESSAGE_CENTER_TEST)
gtest_discover_tests(COMMUNICATION_TEST)
gtest_discover_tests(FIFO_TEST)