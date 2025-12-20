add_subdirectory(comms)
add_subdirectory(maths)
add_subdirectory(control)
add_subdirectory(attitude)

add_library(FIFO INTERFACE)
target_include_directories(FIFO INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(FIFO INTERFACE LIB::UARM_LIB)
add_library(ALGORITHMS::FIFO ALIAS FIFO)