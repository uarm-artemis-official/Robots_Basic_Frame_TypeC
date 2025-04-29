add_library(ATTITUDE_RESOLUTION_LIBRARIES STATIC angle_process.c mahony_ahrs.c madgwick_ahrs.c)
target_link_libraries(ATTITUDE_RESOLUTION_LIBRARIES 
    PRIVATE 
        ARM_MATH
        MATHS_LIBRARIES)
target_include_directories(ATTITUDE_RESOLUTION_LIBRARIES
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR})