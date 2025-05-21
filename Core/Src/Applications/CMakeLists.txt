add_library(APPS_INCLUDE INTERFACE)
target_include_directories(APPS_INCLUDE
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ALGORITHMS_PATH}/control
        ${ALGORITHMS_PATH}/attitude
        ${SUBSYSTEMS_PATH}
        ${LIB_PATH}
        ${FIRMWARE_PATH}/Device/motors)

add_library(CHASSIS_DRIVES STATIC Omni_Drive.cpp Swerve_Drive.cpp)

# Add Shoot_App.c back again later
add_library(BASIC_APPS STATIC Chassis_App.cpp Gimbal_App.cpp)
target_include_directories(BASIC_APPS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BASIC_APPS
    PUBLIC
        APPS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_OS
        LIB::UARM_MATH
        SUBSYSTEMS::MESSAGE_CENTER
        SUBSYSTEMS::MOTORS
        ALGORITHMS::ATTITUDE_LIB
        ALGORITHMS::CONTROL_LIB)

add_library(GENERIC_APPS 
    STATIC 
        Comm_App.cpp
        Control_App.cpp
        IMU_App.cpp
        PC_UART_App.cpp
        Referee_App.c
        Timer_App.cpp
        WatchDog_App.c)
target_include_directories(GENERIC_APPS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(GENERIC_APPS 
    PUBLIC 
        APPS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_OS
        LIB::UARM_MATH
        SUBSYSTEMS::MESSAGE_CENTER
        SUBSYSTEMS::EVENT_CENTER
        SUBSYSTEMS::MOTORS
        SUBSYSTEMS::CAN_COMM
        SUBSYSTEMS::PC_COMM
        SUBSYSTEMS::IMU
        SUBSYSTEMS::DEBUG
        ALGORITHMS::COMMS_LIB)