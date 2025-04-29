add_library(USER_SOFTWARE_LIBRARIES 
    STATIC
        comms.c
        debugger.c
        error_handler.c
        event_center.c
        message_center.c
        pack_handler.c
        redirection.c
        referee_ui.c
        self_check.c)
target_include_directories(USER_SOFTWARE_LIBRARIES PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(USER_SOFTWARE_LIBRARIES 
    PUBLIC
        COMMON_INCLUDES
        SSD1306_DRIVER
        DWT)