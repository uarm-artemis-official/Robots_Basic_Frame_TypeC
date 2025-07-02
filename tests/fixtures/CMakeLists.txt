add_library(CHASSIS_APP_FIXTURE INTERFACE)
target_include_directories(CHASSIS_APP_FIXTURE 
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CHASSIS_APP_FIXTURE 
    INTERFACE 
        BASIC_APPS
        CHASSIS_DRIVES
        GTest::gtest)