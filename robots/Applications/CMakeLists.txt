add_library(APPS_INCLUDE INTERFACE)
target_include_directories(APPS_INCLUDE
    INTERFACE
        ${CONFIGS_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ALGORITHMS_PATH}/control
        ${ALGORITHMS_PATH}/attitude
        ${SUBSYSTEMS_PATH}
        ${MODULES_PATH}
        ${LIB_PATH}
        ${ROBOTS_PATH}
        ${LIB_PATH}/Middleware
        ${FIRMWARE_PATH}/Device/motors)

add_library(CHASSIS_DRIVES STATIC Omni_Drive.cpp Swerve_Drive.cpp)
target_link_libraries(CHASSIS_DRIVES 
    PUBLIC
        APPS_INCLUDE
        ALGORITHMS::CONTROL_LIB
        SUBSYSTEMS::SUBSYSTEMS_INCLUDE
        ROBOTS::MESSAGES
        SUBSYSTEMS::COMMUNICATION
        ISRS::CAN_ISR
        ISRS::UART_ISR
        LIB::UARM_LIB
        LIB::UARM_MATH)

add_library(BASIC_APPS STATIC Chassis_App.cpp Gimbal_App.cpp Shoot_App.cpp)
target_link_libraries(BASIC_APPS
    PUBLIC
        APPS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_MATH
        CHASSIS_DRIVES
        ROBOTS::MESSAGES
        ISRS::CAN_ISR
        ISRS::UART_ISR
        MODULES::MESSAGE_CENTER
        ALGORITHMS::ATTITUDE_LIB
        SUBSYSTEMS::SUBSYSTEMS_INCLUDE
        SUBSYSTEMS::COMMUNICATION
        ALGORITHMS::CONTROL_LIB)

add_library(GENERIC_APPS 
    STATIC 
        Comm_App.cpp
        RC_App.cpp
        IMU_App.cpp
        Referee_App.cpp
        Timer_App.cpp)
target_include_directories(GENERIC_APPS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(GENERIC_APPS 
    PUBLIC 
        APPS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_MATH
        ISRS::CAN_ISR
        ISRS::UART_ISR
        MODULES::MESSAGE_CENTER
        ROBOTS::MESSAGES
        SUBSYSTEMS::SUBSYSTEMS_INCLUDE
        SUBSYSTEMS::COMMUNICATION
        ALGORITHMS::COMMS_LIB
        ISRS::CAN_ISR)