add_library(SSD1306_DRIVER2 STATIC ssd1306_driver.cpp)
target_include_directories(SSD1306_DRIVER2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(SSD1306_DRIVER2 PUBLIC MW::MIDDLEWARE)
add_library(FW::DEVICE::SSD1306_DRIVER2 ALIAS SSD1306_DRIVER2)

add_library(BUZZER STATIC buzzer.cpp)
target_include_directories(BUZZER PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BUZZER PUBLIC MW::MIDDLEWARE)

add_library(ISOTP INTERFACE)
target_include_directories(ISOTP INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ISOTP INTERFACE LIB::UARM_LIB)
add_library(MODULES::ISOTP ALIAS ISOTP)

add_library(MESSAGE_CENTER INTERFACE)
target_include_directories(MESSAGE_CENTER INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MESSAGE_CENTER 
    INTERFACE
        SUBSYSTEMS_INCLUDE
        LIB::UARM_OS
        LIB::UARM_LIB
        MW::MIDDLEWARE)
add_library(MODULES::MESSAGE_CENTER ALIAS MESSAGE_CENTER)

add_library(CAN2_TP INTERFACE)
target_include_directories(CAN2_TP INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CAN2_TP 
    INTERFACE 
        LIB::UARM_LIB)
add_library(MODULES::CAN2_TP ALIAS CAN2_TP)

add_library(UC_UART INTERFACE)
target_include_directories(UC_UART INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(UC_UART INTERFACE LIB::UARM_LIB)
add_library(MODULES::UC_UART ALIAS UC_UART)

add_library(SIMPLE_COMM INTERFACE)
target_include_directories(SIMPLE_COMM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(SIMPLE_COMM 
    INTERFACE 
        ALGORITHMS::FIFO
        LIB::UARM_LIB
        MW::MIDDLEWARE)
add_library(MODULES::SIMPLE_COMM ALIAS SIMPLE_COMM)