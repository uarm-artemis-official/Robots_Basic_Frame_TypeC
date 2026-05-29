add_library(SUBSYSTEMS_INCLUDE INTERFACE)
target_include_directories(SUBSYSTEMS_INCLUDE 
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${MODULES_PATH}
        ${MIDDLEWARE_PATH}
        ${FIRMWARE_PATH}/Device/motors
        ${FIRMWARE_PATH}
        ${ALGORITHMS_PATH}/attitude)
target_link_libraries(SUBSYSTEMS_INCLUDE
    INTERFACE
        LIB::UARM_INCLUDE
        LIB::UARM_MATH)
add_library(SUBSYSTEMS::SUBSYSTEMS_INCLUDE ALIAS SUBSYSTEMS_INCLUDE)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL "GTest")
add_library(IMU STATIC imu.cpp)
target_link_libraries(IMU
    PUBLIC
        SUBSYSTEMS_INCLUDE
        MW::MIDDLEWARE
        LIB::UARM_LIB
        MODULES::BMI088_DRIVER
        MODULES::IST8310_DRIVER
        ALGORITHMS::ATTITUDE_LIB)
add_library(SUBSYSTEMS::IMU ALIAS IMU)


add_library(EVENT_CENTER STATIC event_center.cpp)
target_link_libraries(EVENT_CENTER 
    PUBLIC 
        SUBSYSTEMS_INCLUDE
    MW::MIDDLEWARE
        LIB::UARM_LIB)
add_library(SUBSYSTEMS::EVENT_CENTER ALIAS EVENT_CENTER)


add_library(MOTORS STATIC motors.cpp)
target_link_libraries(MOTORS
    PUBLIC 
        SUBSYSTEMS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_INCLUDE
    MODULES::DJI_MOTOR
    MODULES::LK_MOTOR
        SERVICES::CAN_ISR)
add_library(SUBSYSTEMS::MOTORS ALIAS MOTORS)

add_library(REF_UI STATIC referee_ui.cpp)
target_link_libraries(REF_UI
    PUBLIC
        SUBSYSTEMS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_INCLUDE
        ALGORITHMS::COMMS_LIB)
add_library(SUBSYSTEMS::REF_UI ALIAS REF_UI)

add_library(RC_COMM STATIC rc_comm.cpp)
target_include_directories(RC_COMM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(RC_COMM
    PUBLIC
        SUBSYSTEMS_INCLUDE)
add_library(SUBSYSTEMS::RC_COMM ALIAS RC_COMM)


add_library(AMMO_LID STATIC ammo_lid.cpp)
target_include_directories(AMMO_LID PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(AMMO_LID 
    PUBLIC 
        SUBSYSTEMS_INCLUDE
        LIB::UARM_LIB)
add_library(SUBSYSTEMS::AMMO_LID ALIAS AMMO_LID)
endif()

add_library(COMMUNICATION INTERFACE)
target_include_directories(COMMUNICATION INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(COMMUNICATION 
    INTERFACE 
        MW::MIDDLEWARE
        LIB::UARM_LIB
        MODULES::MESSAGE_CENTER
        MODULES::SIMPLE_COMM)
add_library(SUBSYSTEMS::COMMUNICATION ALIAS COMMUNICATION)