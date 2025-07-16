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

# Setup ARM GNU tools
set(CMAKE_OBJDUMP arm-gnu-objdump)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME pq_nse2025_main)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add CMSIS-DSP library
set(CMSISCORE ${CMAKE_SOURCE_DIR}/Drivers/CMSIS)
add_subdirectory(${CMAKE_SOURCE_DIR}/cmsis-dsp/Source)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/cmsis-dsp/Include)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
    
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "-u _printf_float")


# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_SOURCE_DIR}/Core/devices/flash/flash.c
    ${CMAKE_SOURCE_DIR}/Core/devices/flash/w25qx.c
    ${CMAKE_SOURCE_DIR}/Core/devices/flash/lfs.c
    ${CMAKE_SOURCE_DIR}/Core/devices/codec/transmission.c
    ${CMAKE_SOURCE_DIR}/Core/devices/imu/imu.c
    ${CMAKE_SOURCE_DIR}/Core/devices/imu/bno055.c
    ${CMAKE_SOURCE_DIR}/Core/devices/imu/bno055_support.c
    ${CMAKE_SOURCE_DIR}/Core/devices/environment/environment.c
    ${CMAKE_SOURCE_DIR}/Core/devices/environment/bme280.c
    ${CMAKE_SOURCE_DIR}/Core/devices/environment/bme280_support.c
    ${CMAKE_SOURCE_DIR}/Core/devices/wireless/E22_900T22S.c
    ${CMAKE_SOURCE_DIR}/Core/devices/wireless/wireless.c
    ${CMAKE_SOURCE_DIR}/Core/devices/software/cli.c
    ${CMAKE_SOURCE_DIR}/Core/devices/software/state.c
    ${CMAKE_SOURCE_DIR}/Core/devices/software/apogee.c  
    ${CMAKE_SOURCE_DIR}/Core/devices/software/logger.c
    ${CMAKE_SOURCE_DIR}/Core/devices/battery/battery.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/Core/devices/flash/
    ${CMAKE_SOURCE_DIR}/Core/devices/codec/
    ${CMAKE_SOURCE_DIR}/Core/devices/imu/
    ${CMAKE_SOURCE_DIR}/Core/devices/gnss/
    ${CMAKE_SOURCE_DIR}/Core/devices/environment/
    ${CMAKE_SOURCE_DIR}/Core/devices/wireless/
    ${CMAKE_SOURCE_DIR}/Core/devices/software/
    ${CMAKE_SOURCE_DIR}/Core/devices/battery/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP

    # Add user defined libraries
)
