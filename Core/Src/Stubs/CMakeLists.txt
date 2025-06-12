add_library(SUBSYSTEMS_MOCKS subsystems_mocks.cpp)
target_include_directories(SUBSYSTEMS_MOCKS 
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/Core/Src/Subsystems
        ${PROJECT_SOURCE_DIR}/Core/Src/Firmware/Device/motors
        ${PROJECT_SOURCE_DIR}/Core/Src/Algorithms/attitude)
target_link_libraries(SUBSYSTEMS_MOCKS
    PUBLIC 
        LIB::UARM_INCLUDE
        GTest::gtest 
        GTest::gmock)