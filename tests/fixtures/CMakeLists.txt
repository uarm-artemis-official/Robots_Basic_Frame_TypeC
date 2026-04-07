add_library(CHASSIS_APP_FIXTURE INTERFACE)
target_include_directories(CHASSIS_APP_FIXTURE 
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CHASSIS_APP_FIXTURE 
    INTERFACE 
        BASIC_APPS
        CHASSIS_DRIVES
        GTest::gtest)

add_library(COMMUNICATION_SUBMODULE_FIXTURE INTERFACE)
target_include_directories(COMMUNICATION_SUBMODULE_FIXTURE
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(COMMUNICATION_SUBMODULE_FIXTURE
    INTERFACE
        SUBSYSTEMS::COMMUNICATION
        LIB::FAKE_MESSAGE_CENTER
        MIDDLEWARE_MOCKS
        GTest::gtest)

# add_library(MULTIBOARD_FIXTURE INTERFACE)
# target_include_directories(MULTIBOARD_FIXTURE 
#     INTERFACE
#         ${CMAKE_CURRENT_SOURCE_DIR})
# target_link_libraries(MULTIBOARD_FIXTURE 
#     INTERFACE 
#         BASIC_APPS
#         CHASSIS_DRIVES
#         GENERIC_APPS
#         MIDDLEWARE_MOCKS
#         SUBSYSTEMS_MOCKS
#         ISRS::CAN_ISR
#         FAKE_MESSAGE_CENTER
#         GTest::gtest)