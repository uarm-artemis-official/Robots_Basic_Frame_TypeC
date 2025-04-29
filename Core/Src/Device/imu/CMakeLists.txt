add_library(BMI088_DRIVER STATIC bmi088_driver.c bmi088_middleware.c)
target_include_directories(BMI088_DRIVER PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BMI088_DRIVER PUBLIC COMMON_INCLUDES)

add_library(IST8310_DRIVER STATIC ist8310driver_middleware.c ist8310driver.c)
target_include_directories(IST8310_DRIVER PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IST8310_DRIVER PUBLIC COMMON_INCLUDES)

add_library(IMU STATIC imu.c)
target_include_directories(IMU PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(IMU 
    PUBLIC
        COMMON_INCLUDES
        ATTITUDE_RESOLUTION_LIBRARIES
        CONTROL_LIBRARIES)
