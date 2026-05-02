cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Set middleware and MCU platform options for this base.
set(PLATFORM_MCU "STM32F427XX" CACHE STRING "Target MCU for the build")
set(ENABLE_CAN_MIDDLEWARE ON CACHE BOOL "Enable CAN middleware for the project")
set(ENABLE_GPIO_MIDDLEWARE ON CACHE BOOL "Enable GPIO middleware for the project")
set(ENABLE_UART_MIDDLEWARE ON CACHE BOOL "Enable UART middleware for the project")