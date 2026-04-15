add_library(MESSAGES INTERFACE)
target_include_directories(MESSAGES INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(MESSAGES INTERFACE LIB::UARM_LIB)
add_library(ROBOTS::MESSAGES ALIAS MESSAGES)


add_subdirectory(Applications)
# if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
#     add_subdirectory(Applications)
# else()
#     add_subdirectory(Applications)
# endif()
