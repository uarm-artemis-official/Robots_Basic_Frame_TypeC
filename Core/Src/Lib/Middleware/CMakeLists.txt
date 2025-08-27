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
    target_link_libraries(MIDDLEWARE 
        INTERFACE 
            TEST_MIDDLEWARE)
endif()
