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

add_library(UC_UARTV1 STATIC uc_uartv1.cpp)
target_include_directories(UC_UARTV1 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(UC_UARTV1 
    PUBLIC 
        LIB::UARM_LIB
        MW::MIDDLEWARE)

add_library(MESSAGE_CENTER STATIC message_center.cpp)
target_include_directories(MESSAGE_CENTER PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MESSAGE_CENTER 
    PUBLIC 
        SUBSYSTEMS_INCLUDE
        LIB::UARM_OS
        LIB::UARM_LIB
        MW::MIDDLEWARE)
add_library(MODULES::MESSAGE_CENTER ALIAS MESSAGE_CENTER)
