add_library(COMMS_LIB STATIC crc.c)
target_include_directories(COMMS_LIB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(COMMS_LIB PUBLIC UARM_INCLUDE)
add_library(ALGORITHMS::COMMS_LIB ALIAS COMMS_LIB)