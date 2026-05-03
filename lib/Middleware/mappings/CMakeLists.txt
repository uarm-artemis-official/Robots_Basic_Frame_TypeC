add_library(MAPPER)
target_include_directories(MAPPER PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if (BASE_TYPE STREQUAL "robots3v3")
    target_sources(MAPPER 
        PRIVATE 
            ${CMAKE_CURRENT_SOURCE_DIR}/robots3v3_mapping.cpp)
elseif (BASE_TYPE STREQUAL "engineer")
    target_sources(MAPPER 
        PRIVATE 
            ${CMAKE_CURRENT_SOURCE_DIR}/engineer_mapping.cpp)
elseif (BASE_TYPE STREQUAL "telemetry")
    target_sources(MAPPER 
        PRIVATE 
            ${CMAKE_CURRENT_SOURCE_DIR}/telemetry_system_mapping.cpp)
else()
    message(FATAL_ERROR "Unsupported base type: ${BASE_TYPE}")
endif()
