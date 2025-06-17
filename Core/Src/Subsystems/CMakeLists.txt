add_library(SUBSYSTEMS_INCLUDE INTERFACE)
target_include_directories(SUBSYSTEMS_INCLUDE 
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${FIRMWARE_PATH}/Device/motors
        ${FIRMWARE_PATH}/Device/bmi088
        ${FIRMWARE_PATH}/Device/ist8310
        ${FIRMWARE_PATH}
        ${ALGORITHMS_PATH}/attitude)
add_library(SUBSYSTEMS::SUBSYSTEMS_INCLUDE ALIAS SUBSYSTEMS_INCLUDE)


add_library(DEBUG STATIC debug.cpp)
target_include_directories(DEBUG 
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(DEBUG
    PUBLIC
        SUBSYSTEMS_INCLUDE
        LIB::UARM_LIB)
add_library(SUBSYSTEMS::DEBUG ALIAS DEBUG)


add_library(IMU STATIC imu.cpp)
target_link_libraries(IMU
    PUBLIC
        SUBSYSTEMS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_OS
        FW::DEVICE::BMI088_DRIVER
        FW::DEVICE::IST8310_DRIVER
        ALGORITHMS::ATTITUDE_LIB)
add_library(SUBSYSTEMS::IMU ALIAS IMU)


add_library(MESSAGE_CENTER STATIC message_center.cpp)
target_link_libraries(MESSAGE_CENTER 
    PUBLIC 
        SUBSYSTEMS_INCLUDE
        LIB::UARM_OS
        LIB::UARM_LIB)
add_library(SUBSYSTEMS::MESSAGE_CENTER ALIAS MESSAGE_CENTER)


add_library(EVENT_CENTER STATIC event_center.cpp)
target_link_libraries(EVENT_CENTER 
    PUBLIC 
        SUBSYSTEMS_INCLUDE
        LIB::UARM_OS
        LIB::UARM_LIB)
add_library(SUBSYSTEMS::EVENT_CENTER ALIAS EVENT_CENTER)


add_library(MOTORS STATIC motors.cpp)
target_link_libraries(MOTORS
    PUBLIC 
        FW::DEVICE::MOTOR_LIB
        LIB::UARM_LIB
        LIB::UARM_INCLUDE
        ISRS::CAN_ISR)
add_library(SUBSYSTEMS::MOTORS ALIAS MOTORS)


add_library(PC_COMM STATIC pc_comm.cpp)
target_link_libraries(PC_COMM
    PUBLIC
        LIB::UARM_LIB
        LIB::UARM_INCLUDE
        FW::PACK_HANDLER)
add_library(SUBSYSTEMS::PC_COMM ALIAS PC_COMM)


add_library(CAN_COMM STATIC can_comm.cpp)
target_include_directories(CAN_COMM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CAN_COMM 
    PUBLIC
        SUBSYSTEMS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_OS
        FW::QUEUE_M)
add_library(SUBSYSTEMS::CAN_COMM ALIAS CAN_COMM)

add_library(REF_UI STATIC referee_ui.cpp)
target_link_libraries(REF_UI
    PUBLIC
        SUBSYSTEMS_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_INCLUDE
        ALGORITHMS::COMMS_LIB
        FW::REF_DATA)
add_library(SUBSYSTEMS::REF_UI ALIAS REF_UI)