add_library(MIDDLEWARE_MOCKS INTERFACE)
target_include_directories(MIDDLEWARE_MOCKS 
    INTERFACE
        ${PROJECT_SOURCE_DIR}/lib/Middleware
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MIDDLEWARE_MOCKS
    INTERFACE
        GTest::gtest
        GTest::gmock)

add_library(SUBSYSTEMS_MOCKS INTERFACE)
target_include_directories(SUBSYSTEMS_MOCKS 
    INTERFACE
        ${PROJECT_SOURCE_DIR}/Core/Src/Subsystems
        ${PROJECT_SOURCE_DIR}/Core/Src/Firmware/Device/motors
        ${PROJECT_SOURCE_DIR}/Core/Src/Algorithms/attitude
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(SUBSYSTEMS_MOCKS
    INTERFACE
        LIB::UARM_INCLUDE
        GTest::gtest 
        GTest::gmock)

