add_library(TOPICS STATIC topics.cpp)
target_include_directories(TOPICS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(TOPICS 
    PUBLIC 
        MODULES::MESSAGE_CENTER
        SUBSYSTEMS_INCLUDE)
add_library(ROBOTS::TOPICS ALIAS TOPICS)


if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    add_subdirectory(Applications)
else()
    add_subdirectory(Firmware)
    add_subdirectory(Applications)
endif()
