cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


set(INPUT_)

# Define the build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()


if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Test)
    include("cmake/mingw64-gcc.cmake") # toolchain file
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Swerve")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Swerve)
    include("cmake/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_SWERVE "-O0 -g3 -DSWERVE_CHASSIS -DSWERVE_GIMBAL")
    set(CMAKE_CXX_FLAGS_SWERVE "-O0 -g3 -DSWERVE_CHASSIS -DSWERVE_GIMBAL")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Omni")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Omni)
    include("cmake/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_OMNI "-O0 -g3 -DOMNI_CHASSIS -DOMNI_GIMBAL")
    set(CMAKE_CXX_FLAGS_OMNI "-O0 -g3 -DOMNI_CHASSIS -DOMNI_GIMBAL")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Mecanum")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Mecanum)
    include("cmake/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_MECANUM "-O0 -g3 -DMECANUM_CHASSIS -DMECANUM_GIMBAL")
    set(CMAKE_CXX_FLAGS_MECANUM "-O0 -g3 -DMECANUM_CHASSIS -DMECANUM_GIMBAL")
else()
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023)
    include("cmake/gcc-arm-none-eabi.cmake") # toolchain file
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C CXX ASM)

if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    include(CTest)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    add_library(COMMON_INCLUDES INTERFACE)
    target_include_directories(COMMON_INCLUDES
        INTERFACE
            ${PROJECT_SOURCE_DIR}/Core/Inc)
    target_compile_definitions(COMMON_INCLUDES 
        INTERFACE 
            GTEST)

    add_subdirectory(${PROJECT_SOURCE_DIR}/Core/Src)
    add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
else()
    add_executable(${CMAKE_PROJECT_NAME})

    add_subdirectory(cmake/stm32cubemx)
    add_subdirectory(${PROJECT_SOURCE_DIR}/Core/Src)

    add_library(COMMON_INCLUDES INTERFACE)
    target_link_libraries(COMMON_INCLUDES
        INTERFACE
            stm32cubemx)
    target_compile_definitions(COMMON_INCLUDES
        INTERFACE
            ARM_MATH_CM4)

    # Link directories setup
    target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
    )

    # Add sources to executable
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        ${PROJECT_SOURCE_DIR}/Core/Src/main.cpp
    )

    # Add include paths
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        ${PROJECT_SOURCE_DIR}/Core/Inc
    )

    # Add project symbols (macros)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
    )

    # Add linked libraries
    target_link_libraries(${CMAKE_PROJECT_NAME}
        BASIC_APPS
        GENERIC_APPS
        ISRS::UART_ISR
        ISRS::CAN_ISR
        SUBSYSTEMS::DEBUG
        SUBSYSTEMS::RC_COMM
        SUBSYSTEMS::AMMO_LID
        SUBSYSTEMS::PC_COMM
        SUBSYSTEMS::CAN_COMM
        SUBSYSTEMS::MOTORS
        SUBSYSTEMS::MESSAGE_CENTER
        SUBSYSTEMS::EVENT_CENTER
        SUBSYSTEMS::IMU
        stm32cubemx)
endif()
