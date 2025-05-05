add_library(CONTROL_LIBRARIES STATIC feedforward.c fuzzy_control.c pid.c ramp.c)
target_link_libraries(CONTROL_LIBRARIES 
    PUBLIC 
        MATHS_LIBRARIES 
        COMMON_INCLUDES)
target_include_directories(CONTROL_LIBRARIES PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})