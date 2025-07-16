add_library(CONTROL_LIB STATIC feedforward.cpp pid.cpp ramp.cpp)
target_link_libraries(CONTROL_LIB 
    PUBLIC 
        LIB::UARM_INCLUDE
        LIB::UARM_LIB
        LIB::UARM_MATH)
target_include_directories(CONTROL_LIB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_library(ALGORITHMS::CONTROL_LIB ALIAS CONTROL_LIB)