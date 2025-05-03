add_library(FAKE_ARM_MATH STATIC fake_arm_math.c)
target_include_directories(FAKE_ARM_MATH 
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR})

add_library(ARM_MATH INTERFACE)
target_include_directories(ARM_MATH 
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ARM_MATH 
    INTERFACE
        FAKE_ARM_MATH)