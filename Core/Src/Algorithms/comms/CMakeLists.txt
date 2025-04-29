add_library(COMMS_LIBRARIES STATIC crc.c queue_m.c)
target_include_directories(COMMS_LIBRARIES
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(COMMS_LIBRARIES
    PUBLIC
        COMMON_INCLUDES)