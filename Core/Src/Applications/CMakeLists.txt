add_library(BASIC_APPS STATIC Chassis_App.c Gimbal_App.c Shoot_App.c)
target_include_directories(BASIC_APPS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BASIC_APPS
    PUBLIC
        COMMON_INCLUDES 
        USER_SOFTWARE_LIBRARIES
        DEVICE_LIBRARIES
        ARM_MATH
        MATHS_LIBRARIES)

add_library(GENERIC_APPS 
    STATIC 
        Comm_App.c 
        Control_App.c 
        IMU_App.c 
        PC_UART_App.c 
        Referee_App.c
        Timer_App.c
        WatchDog_App.c)
target_include_directories(GENERIC_APPS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(GENERIC_APPS 
    PUBLIC 
        COMMON_INCLUDES
        DEVICE_LIBRARIES
        COMMS_LIBRARIES)

# if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
#     add_executable(CHASSIS_APP_TEST Chassis_App_Test.cpp)
#     target_link_directories(CHASSIS_APP_TEST 
#         PRIVATE
#             ${PROJECT_SOURCE_DIR}/Core/Src/Device)
#     target_link_libraries(CHASSIS_APP_TEST 
#         PRIVATE 
#             BASIC_APPS
#             GTest::gtest_main)

#     include(GoogleTest)
#     gtest_discover_tests(CHASSIS_APP_TEST)
# endif()