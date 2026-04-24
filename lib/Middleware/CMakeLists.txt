add_library(MIDDLEWARE INTERFACE)
target_include_directories(MIDDLEWARE 
    INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR})
add_library(MW::MIDDLEWARE ALIAS MIDDLEWARE)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    add_library(DJI_TYPEC_MIDDLEWARE STATIC dji_typec_middleware.cpp)
    target_include_directories(DJI_TYPEC_MIDDLEWARE 
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR})

    if(BASE_TYPE STREQUAL "robots3v3")
        target_compile_definitions(DJI_TYPEC_MIDDLEWARE
            PUBLIC
                MW_ENABLE_GPIO
                MW_ENABLE_CAN
                MW_ENABLE_UART
                MW_ENABLE_I2C
                MW_ENABLE_SPI
                MW_ENABLE_TIM
                MW_ENABLE_RTOS)
    elseif(BASE_TYPE STREQUAL "telemetry")
        target_compile_definitions(DJI_TYPEC_MIDDLEWARE
            PUBLIC
                MW_ENABLE_GPIO
                MW_ENABLE_CAN)
    else()
        message(FATAL_ERROR "Unsupported BASE_TYPE for middleware selection: ${BASE_TYPE}")
    endif()

    target_link_libraries(DJI_TYPEC_MIDDLEWARE 
        PUBLIC 
            COMMON_INCLUDES
            LIB::UARM_LIB)
    
    target_link_libraries(MIDDLEWARE 
        INTERFACE 
            FreeRTOS
            DJI_TYPEC_MIDDLEWARE)
else()
    add_library(TEST_MIDDLEWARE STATIC test_middleware.cpp)
    target_include_directories(TEST_MIDDLEWARE 
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(TEST_MIDDLEWARE 
        PUBLIC 
            LIB::UARM_LIB)

    target_link_libraries(MIDDLEWARE 
        INTERFACE 
            TEST_MIDDLEWARE)
endif()
