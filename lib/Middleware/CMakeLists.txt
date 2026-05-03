add_library(MIDDLEWARE INTERFACE)
target_include_directories(MIDDLEWARE 
    INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR})
add_library(MW::MIDDLEWARE ALIAS MIDDLEWARE)

if (PLATFORM_MCU)
    if (PLATFORM_MCU STREQUAL "STM32F407XX" OR PLATFORM_MCU STREQUAL "STM32F427XX")
        add_subdirectory(mappings)
    else()
        message(FATAL_ERROR "Unsupported MCU: ${PLATFORM_MCU}")
    endif()
endif()

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    add_library(DJI_TYPEC_MIDDLEWARE STATIC dji_typec_middleware.cpp)
    target_include_directories(DJI_TYPEC_MIDDLEWARE 
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR})

    target_link_libraries(DJI_TYPEC_MIDDLEWARE 
        PUBLIC 
            COMMON_INCLUDES
            LIB::UARM_LIB)
    
    target_link_libraries(MIDDLEWARE 
        INTERFACE 
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
