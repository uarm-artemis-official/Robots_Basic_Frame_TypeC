if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    add_library(ISR_INTERFACES INTERFACE)
    target_include_directories(ISR_INTERFACES INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(ISR_INTERFACES INTERFACE LIB::UARM_LIB)
    add_library(ISRS::ISR_INTERFACES ALIAS ISR_INTERFACES)
else()
    add_library(ISR_INTERFACES STATIC dji_install_isrs.cpp)
    target_include_directories(ISR_INTERFACES PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(ISR_INTERFACES 
        PUBLIC 
            LIB::UARM_LIB
            MW::MIDDLEWARE)
    add_library(ISRS::ISR_INTERFACES ALIAS ISR_INTERFACES)
endif()

add_library(CAN_ISR INTERFACE)
target_include_directories(CAN_ISR INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CAN_ISR
    INTERFACE
        ISRS::ISR_INTERFACES
        MW::MIDDLEWARE)
add_library(ISRS::CAN_ISR ALIAS CAN_ISR)

add_library(UART_ISR INTERFACE)
target_include_directories(UART_ISR INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(UART_ISR
    INTERFACE
        ISRS::ISR_INTERFACES
        MW::MIDDLEWARE)
add_library(ISRS::UART_ISR ALIAS UART_ISR)