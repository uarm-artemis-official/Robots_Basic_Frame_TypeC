if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
else()
    add_library(CAN_ISR STATIC can_isr.cpp)
    target_include_directories(CAN_ISR PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(CAN_ISR 
        PUBLIC 
            SUBSYSTEMS::SUBSYSTEMS_INCLUDE
            FW::DEVICE::MOTOR_LIB
            ROBOTS::TOPICS
            SUBSYSTEMS::MOTORS
            MODULES::MESSAGE_CENTER)
    add_library(ISRS::CAN_ISR ALIAS CAN_ISR)


    add_library(UART_ISR STATIC uart_isr.cpp)
    target_include_directories(UART_ISR PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(UART_ISR
        PUBLIC
            COMMON_INCLUDES
            ROBOTS::TOPICS
            MODULES::MESSAGE_CENTER
            FW::PACK_HANDLER)
    add_library(ISRS::UART_ISR ALIAS UART_ISR)
endif()