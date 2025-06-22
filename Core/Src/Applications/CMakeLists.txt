add_library(APPS_INCLUDE INTERFACE)
target_include_directories(APPS_INCLUDE
    INTERFACE
        ${PROJECT_SOURCE_DIR}/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${ALGORITHMS_PATH}/control
        ${ALGORITHMS_PATH}/attitude
        ${SUBSYSTEMS_PATH}
        ${LIB_PATH}
        ${FIRMWARE_PATH}/Device/motors)

add_library(CHASSIS_DRIVES STATIC Omni_Drive.cpp Swerve_Drive.cpp)
target_link_libraries(CHASSIS_DRIVES 
    PUBLIC
        APPS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_OS
        LIB::UARM_MATH)

# Add Shoot_App.c back again later
add_library(BASIC_APPS STATIC Chassis_App.cpp Gimbal_App.cpp Shoot_App.cpp)
target_link_libraries(BASIC_APPS
    PUBLIC
        APPS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_OS
        LIB::UARM_MATH
        CHASSIS_DRIVES
        ALGORITHMS::ATTITUDE_LIB
        SUBSYSTEMS::SUBSYSTEMS_INCLUDE
        ALGORITHMS::CONTROL_LIB)

add_library(GENERIC_APPS 
    STATIC 
        Comm_App.cpp
        RC_App.cpp
        IMU_App.cpp
        PC_UART_App.cpp
        Referee_App.cpp
        Timer_App.cpp
        WatchDog_App.c)
target_include_directories(GENERIC_APPS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(GENERIC_APPS 
    PUBLIC 
        APPS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_OS
        LIB::UARM_MATH
        SUBSYSTEMS_INCLUDE
        ALGORITHMS::COMMS_LIB)