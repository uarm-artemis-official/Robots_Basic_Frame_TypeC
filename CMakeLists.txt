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
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define the build type
if (NOT CMAKE_BUILD_TYPE)
    message(FATAL_ERROR "CMAKE_BUILD_TYPE must be set.")
endif()

if (PLATFORM_MCU)
    if (PLATFORM_MCU STREQUAL "STM32F407XX")
        message("Setting up: STM32F407XX")
        add_compile_definitions(STM32F407XX)
    elseif (PLATFORM_MCU STREQUAL "STM32F427XX")
        message("Setting up: STM32F427XX")
        add_compile_definitions(STM32F427XX)
    else()
        message(FATAL_ERROR "Unsupported MCU: ${PLATFORM_MCU}")
    endif()
endif()

if (ENABLE_CAN_MIDDLEWARE)
    add_compile_definitions(MW_ENABLE_CAN)
endif()

if (ENABLE_GPIO_MIDDLEWARE)
    add_compile_definitions(MW_ENABLE_GPIO)
endif()

if (ENABLE_I2C_MIDDLEWARE)
    add_compile_definitions(MW_ENABLE_I2C)
endif()

if (ENABLE_SPI_MIDDLEWARE)
    add_compile_definitions(MW_ENABLE_SPI)
endif()

if (ENABLE_TIM_MIDDLEWARE)
    add_compile_definitions(MW_ENABLE_TIM)
endif()

if (ENABLE_UART_MIDDLEWARE)
    add_compile_definitions(MW_ENABLE_UART)
endif()

if (ENABLE_RTOS_MIDDLEWARE)
    add_compile_definitions(MW_ENABLE_RTOS)
endif()


if (BASE_TYPE STREQUAL "robots3v3")
    set(BASE_DIR "${CMAKE_SOURCE_DIR}/bases/robots3v3")
    add_compile_definitions(ROBOTS3V3)
elseif (BASE_TYPE STREQUAL "telemetry")
    set(BASE_DIR "${CMAKE_SOURCE_DIR}/bases/telemetry_system")
    add_compile_definitions(TELEMETRY_SYSTEM)
elseif (BASE_TYPE STREQUAL "engineer")
    set(BASE_DIR "${CMAKE_SOURCE_DIR}/bases/engineer")
    add_compile_definitions(ENGINEER)
else()
    message("No base type defined")
endif()

set(BASE_CMAKE_DIR "${BASE_DIR}/cmake")

if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Test)
    include("bases/robots3v3/cmake/mingw64-gcc.cmake") # toolchain file
    set(CMAKE_C_FLAGS_GTEST "-O0 -g3 -fprofile-arcs -ftest-coverage -DGTEST")
    set(CMAKE_CXX_FLAGS_GTEST "-O0 -g3 -fprofile-arcs -ftest-coverage  -DGTEST")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Swerve")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Swerve)
    include("${BASE_CMAKE_DIR}/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_SWERVE "-O0 -g3 -DSWERVE_CHASSIS -DSWERVE_GIMBAL -DSENTRY")
    set(CMAKE_CXX_FLAGS_SWERVE "-O0 -g3 -DSWERVE_CHASSIS -DSWERVE_GIMBAL -DSENTRY")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Omni")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Omni)
    include("${BASE_CMAKE_DIR}/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_OMNI "-O0 -g3 -DOMNI_CHASSIS -DOMNI_GIMBAL -DOMNI_INFANTRY -DINFANTRY")
    set(CMAKE_CXX_FLAGS_OMNI "-O0 -g3 -DOMNI_CHASSIS -DOMNI_GIMBAL -DOMNI_INFANTRY -DINFANTRY")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Hero")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Hero)
    include("${BASE_CMAKE_DIR}/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_HERO "-O0 -g3 -DHERO_CHASSIS -DHERO_GIMBAL -DHERO")
    set(CMAKE_CXX_FLAGS_HERO "-O0 -g3 -DHERO_CHASSIS -DHERO_GIMBAL -DHERO")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Auto_Aim_Rig")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Auto_Aim_Rig)
    include("${BASE_CMAKE_DIR}/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_AUTO_AIM_RIG "-O0 -g3 -DAUTO_AIM_RIG")
    set(CMAKE_CXX_FLAGS_AUTO_AIM_RIG "-O0 -g3 -DAUTO_AIM_RIG")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Telemetry")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Telemetry)
    include("${BASE_CMAKE_DIR}/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_TELEMETRY "-O0 -g3 -DTELEMETRY")
    set(CMAKE_CXX_FLAGS_TELEMETRY "-O0 -g3 -DTELEMETRY")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Engineer")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Engineer)
    include("${BASE_CMAKE_DIR}/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_ENGINEER "-O0 -g3 -DENGINEER")
    set(CMAKE_CXX_FLAGS_ENGINEER "-O0 -g3 -DENGINEER")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Kiddie_Mode")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Kiddie_Mode)
    include("${BASE_CMAKE_DIR}/gcc-arm-none-eabi.cmake") # toolchain file
    set(CMAKE_C_FLAGS_KIDDIE_MODE "-O0 -g3 -DKIDDIE_MODE -DINFANTRY")
    set(CMAKE_CXX_FLAGS_KIDDIE_MODE "-O0 -g3 -DKIDDIE_MODE -DINFANTRY")
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Doxygen")
    set(CMAKE_PROJECT_NAME Basic_Frame_TypeC_2023_Doxygen)
else()
    message(FATAL_ERROR "Unrecognized build type: ${CMAKE_BUILD_TYPE}.")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C CXX ASM)


set(ROBOTS_PATH ${PROJECT_SOURCE_DIR}/robots)
set(APPLICATIONS_PATH ${PROJECT_SOURCE_DIR}/robots/Applications)
set(FIRMWARE_PATH ${PROJECT_SOURCE_DIR}/robots/Firmware)
set(CONFIGS_PATH ${PROJECT_SOURCE_DIR}/robots/Configs)
set(ALGORITHMS_PATH ${PROJECT_SOURCE_DIR}/lib/Algorithms)
set(SUBSYSTEMS_PATH ${PROJECT_SOURCE_DIR}/lib/Subsystems)
set(MIDDLEWARE_PATH ${PROJECT_SOURCE_DIR}/lib/Middleware)
set(MODULES_PATH ${PROJECT_SOURCE_DIR}/lib/Modules)
set(ISRS_PATH ${PROJECT_SOURCE_DIR}/lib/ISRs)

if (${CMAKE_BUILD_TYPE} STREQUAL "GTest")
    include(CTest)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
        DOWNLOAD_EXTRACT_TIMESTAMP true
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    add_library(COMMON_INCLUDES INTERFACE)
    target_compile_definitions(COMMON_INCLUDES 
        INTERFACE 
            GTEST)

    add_subdirectory(${PROJECT_SOURCE_DIR}/lib)
    add_subdirectory(${PROJECT_SOURCE_DIR}/robots)
    add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
elseif (${CMAKE_BUILD_TYPE} STREQUAL "Doxygen")
    find_package(Doxygen)

    if (Doxygen_FOUND)
        set(DOXYGEN_GENERATE_HTML YES)
        set(DOXYGEN_GENERATE_MAN NO)
        set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out/doxygen)
        set(DOXYGEN_EXCLUDE_PATTERNS "*/Lib/DSP/*")
        set(DOXYGEN_WARN_IF_UNDOCUMENTED NO)

        doxygen_add_docs(PROJECT_DOCS ${PROJECT_SOURCE_DIR}/Core/Src ALL)
    endif()
else()
    add_executable(${CMAKE_PROJECT_NAME})

    message("(ROOT) Adding Project name: " ${CMAKE_PROJECT_NAME})

    add_subdirectory(${BASE_DIR})
    add_subdirectory(${PROJECT_SOURCE_DIR}/lib)
    add_subdirectory(${PROJECT_SOURCE_DIR}/robots)

    add_library(COMMON_INCLUDES INTERFACE)
    target_link_libraries(COMMON_INCLUDES
        INTERFACE
            stm32cubemx)
    target_compile_definitions(COMMON_INCLUDES
        INTERFACE
            ARM_MATH_CM4)

    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/robots/main.cpp
    )

    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/robots/Configs
    )

    # Add linked libraries
    target_link_libraries(${CMAKE_PROJECT_NAME}
        BASIC_APPS
        GENERIC_APPS
        ISRS::UART_ISR
        ISRS::CAN_ISR
        SUBSYSTEMS::RC_COMM
        SUBSYSTEMS::AMMO_LID
        SUBSYSTEMS::MOTORS
        MODULES::MESSAGE_CENTER
        SUBSYSTEMS::EVENT_CENTER
        SUBSYSTEMS::IMU
        SUBSYSTEMS::REF_UI
        ROBOTS::PLATFORMS
        stm32cubemx)
endif()