add_library(TOPICS INTERFACE)
target_include_directories(TOPICS INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(TOPICS INTERFACE LIB::UARM_LIB)
add_library(ROBOTS::TOPICS ALIAS TOPICS)


if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    add_subdirectory(Applications)
else()
    add_subdirectory(Firmware)
    add_subdirectory(Applications)
endif()
