add_library(ATTITUDE_LIB STATIC angle_process.c mahony_ahrs.c madgwick_ahrs.c)
target_link_libraries(ATTITUDE_LIB 
    PUBLIC
        LIB::UARM_LIB
        LIB::UARM_MATH
        ALGORITHMS::KALMAN_FILTERS)
target_include_directories(ATTITUDE_LIB INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
add_library(ALGORITHMS::ATTITUDE_LIB ALIAS ATTITUDE_LIB)