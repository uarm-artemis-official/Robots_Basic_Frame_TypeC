add_library(BMI088_DRIVER STATIC bmi088_driver.c bmi088_middleware.c)
target_include_directories(BMI088_DRIVER PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BMI088_DRIVER PUBLIC COMMON_INCLUDES)
add_library(FW::DEVICE::BMI088_DRIVER ALIAS BMI088_DRIVER)