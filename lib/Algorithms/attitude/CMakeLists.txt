add_library(ATTITUDE_LIB STATIC madgewick.cpp)
target_link_libraries(ATTITUDE_LIB 
    PUBLIC
        LIB::UARM_LIB
        LIB::UARM_MATH)
target_include_directories(ATTITUDE_LIB INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
add_library(ALGORITHMS::ATTITUDE_LIB ALIAS ATTITUDE_LIB)