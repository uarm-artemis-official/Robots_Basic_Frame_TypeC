add_library(KALMAN_FILTERS STATIC)
target_sources(KALMAN_FILTERS PUBLIC kalman_filters.c)
target_link_libraries(KALMAN_FILTERS PUBLIC LIB::UARM_MATH LIB::UARM_LIB LIB::UARM_INCLUDE)
target_include_directories(KALMAN_FILTERS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_library(ALGORITHMS::KALMAN_FILTERS ALIAS KALMAN_FILTERS)

add_library(DIGITAL_FILTERS STATIC digital_filters.cpp)
target_include_directories(DIGITAL_FILTERS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(DIGITAL_FILTERS PUBLIC LIB::UARM_LIB)
add_library(ALGORITHMS::DIGITAL_FILTERS ALIAS DIGITAL_FILTERS)