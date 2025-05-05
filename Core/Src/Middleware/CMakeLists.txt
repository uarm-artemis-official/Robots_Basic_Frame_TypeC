add_subdirectory(DSP)

add_library(FAKE_ARM_MATH OBJECT "Stubs/fake_arm_math.c")
target_include_directories(FAKE_ARM_MATH PUBLIC Stubs)

add_library(ARM_MATH INTERFACE)
if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    target_link_libraries(ARM_MATH INTERFACE FAKE_ARM_MATH)
else()
    target_link_libraries(ARM_MATH INTERFACE REAL_ARM_MATH)
endif()

add_library(UARM_LIBRARIES STATIC "Src/uarm_math.c" "Src/uarm_os.c")
target_include_directories(UARM_LIBRARIES PUBLIC ${PROJECT_SOURCE_DIR}/Core/Inc)
target_link_libraries(UARM_LIBRARIES PUBLIC ARM_MATH)