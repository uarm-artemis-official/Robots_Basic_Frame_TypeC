
if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    add_subdirectory(Applications)
else()
    add_subdirectory(Firmware)
    add_subdirectory(Applications)
endif()
