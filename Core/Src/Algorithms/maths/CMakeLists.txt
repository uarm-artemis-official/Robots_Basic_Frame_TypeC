add_library(MATHS STATIC "maths.c" "maths.h")
target_link_libraries(MATHS PUBLIC ARM_MATH)
target_include_directories(MATHS PUBLIC ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
                                 PUBLIC ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/Include)

add_library(KALMAN_FILTERS OBJECT "kalman_filters.c" "kalman_filters.h")
target_link_libraries(KALMAN_FILTERS PRIVATE ARM_MATH)
target_include_directories(KALMAN_FILTERS PRIVATE CMSIS_DEVICE_INCLUDES)