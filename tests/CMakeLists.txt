add_executable(FIRST_TEST first_test.cpp)
target_link_libraries(FIRST_TEST PRIVATE GTest::gtest_main)

add_executable(PID_TEST pid_test.cpp)
target_include_directories(PID_TEST
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/Core/Src/Algorithms/maths)
target_link_libraries(PID_TEST 
    PRIVATE 
        GTest::gtest_main
        MATHS_LIBRARIES
        CONTROL_LIBRARIES)

add_executable(MATHS_TEST maths_test.cpp)
target_include_directories(MATHS_TEST 
    PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MATHS_TEST 
    PRIVATE 
        GTest::gtest_main
        MATHS_LIBRARIES)

add_executable(CHASSIS_APP_TEST Chassis_App_Test.cpp)
target_link_directories(CHASSIS_APP_TEST 
    PRIVATE
        ${PROJECT_SOURCE_DIR}/Core/Src/Device)
target_link_libraries(CHASSIS_APP_TEST 
    PRIVATE 
        BASIC_APPS
        GTest::gtest_main)


include(GoogleTest)
gtest_discover_tests(FIRST_TEST)
gtest_discover_tests(PID_TEST)
gtest_discover_tests(MATHS_TEST)
gtest_discover_tests(CHASSIS_APP_TEST)