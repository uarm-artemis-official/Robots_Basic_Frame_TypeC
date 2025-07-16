add_subdirectory(DSP)

add_library(UARM_INCLUDE INTERFACE)
target_include_directories(UARM_INCLUDE INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
add_library(LIB::UARM_INCLUDE ALIAS UARM_INCLUDE)

add_library(UARM_OS STATIC uarm_os.cpp)
target_link_libraries(UARM_OS PUBLIC UARM_INCLUDE)
add_library(LIB::UARM_OS ALIAS UARM_OS)

add_library(UARM_LIB INTERFACE)
target_link_libraries(UARM_LIB INTERFACE UARM_INCLUDE)
add_library(LIB::UARM_LIB ALIAS UARM_LIB)

add_library(UARM_MATH STATIC uarm_math.cpp)
target_link_libraries(UARM_MATH 
    PUBLIC 
        ARM_MATH
        UARM_INCLUDE
        UARM_LIB)
add_library(LIB::UARM_MATH ALIAS UARM_MATH)

if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    target_compile_definitions(UARM_INCLUDE INTERFACE GTEST)
else()
    target_link_libraries(UARM_INCLUDE INTERFACE COMMON_INCLUDES)
    target_link_libraries(UARM_LIB INTERFACE FW::ERROR_HANDLER)
endif()
