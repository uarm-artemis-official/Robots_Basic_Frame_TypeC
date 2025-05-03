add_library(CONTROL_LIBRARIES STATIC feedforward.c fuzzy_control.c pid.c ramp.c)
target_link_libraries(CONTROL_LIBRARIES 
    PUBLIC 
        MATHS_LIBRARIES 
        COMMON_INCLUDES)
target_include_directories(CONTROL_LIBRARIES PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(${CMAKE_BUILD_TYPE} STREQUAL "GTest")
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

    include(GoogleTest)
    gtest_discover_tests(PID_TEST)
endif()