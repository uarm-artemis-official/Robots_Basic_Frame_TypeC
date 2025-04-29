add_subdirectory(imu)
add_subdirectory(oled)

add_library(DEVICE_LIBRARIES STATIC motor.c dwt.c LK_motor.c buzzer.c)
target_include_directories(DEVICE_LIBRARIES PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(DEVICE_LIBRARIES 
    PUBLIC
        COMMON_INCLUDES
        CONTROL_LIBRARIES
        USER_SOFTWARE_LIBRARIES
        IMU
        SSD1306_DRIVER)