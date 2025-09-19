add_library(REAL_ARM_MATH_HARD STATIC IMPORTED)
set_property(TARGET REAL_ARM_MATH_HARD PROPERTY
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/Lib/libarm_cortexM4lf_math.a)

add_library(REAL_ARM_MATH INTERFACE)
target_include_directories(REAL_ARM_MATH INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/Include)
target_link_libraries(REAL_ARM_MATH INTERFACE REAL_ARM_MATH_HARD)

add_library(FAKE_ARM_MATH STATIC Lib/fake_arm_math.c)
target_include_directories(FAKE_ARM_MATH PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Include)

add_library(ARM_MATH INTERFACE)
if(${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    target_link_libraries(ARM_MATH INTERFACE FAKE_ARM_MATH)
else()
    target_link_libraries(ARM_MATH INTERFACE REAL_ARM_MATH)
endif()