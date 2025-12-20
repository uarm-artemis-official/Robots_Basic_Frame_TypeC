add_library(ISR_INTERFACES INTERFACE)
target_include_directories(ISR_INTERFACES INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ISR_INTERFACES INTERFACE LIB::UARM_LIB)
add_library(ISRS::ISR_INTERFACES ALIAS ISR_INTERFACES)

add_library(CAN_ISR STATIC can_isr.cpp)
target_include_directories(CAN_ISR PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CAN_ISR 
    PUBLIC 
        ISRS::ISR_INTERFACES
        MW::MIDDLEWARE)
add_library(ISRS::CAN_ISR ALIAS CAN_ISR)

add_library(UART_ISR STATIC uart_isr.cpp)
target_include_directories(UART_ISR PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(UART_ISR
    PUBLIC
        ISRS::ISR_INTERFACES
        MW::MIDDLEWARE)
add_library(ISRS::UART_ISR ALIAS UART_ISR)