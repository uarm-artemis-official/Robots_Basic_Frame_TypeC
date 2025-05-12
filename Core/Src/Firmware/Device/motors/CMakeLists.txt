add_library(MOTOR_LIB STATIC dji_motor.c lk_motor.c)
target_include_directories(MOTOR_LIB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MOTOR_LIB PUBLIC COMMON_INCLUDES)
add_library(FW::DEVICE::MOTOR_LIB ALIAS MOTOR_LIB)