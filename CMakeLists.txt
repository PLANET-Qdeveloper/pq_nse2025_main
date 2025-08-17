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

# Static Analysis Configuration
option(ENABLE_STATIC_ANALYSIS "Enable static code analysis" OFF)

if(ENABLE_STATIC_ANALYSIS)
    find_program(CPPCHECK_EXECUTABLE cppcheck)
    if(CPPCHECK_EXECUTABLE)
        set(CPPCHECK_OPTIONS
            --enable=all
            --std=c11
            --platform=unix32
            --suppress=missingIncludeSystem
            --suppress=unusedFunction
            --suppress=unmatchedSuppression
            --inline-suppr
            --quiet
            --error-exitcode=1
        )
        
        # Create custom target for static analysis
        add_custom_target(static-analysis
            COMMAND ${CPPCHECK_EXECUTABLE} ${CPPCHECK_OPTIONS} 
                ${CMAKE_SOURCE_DIR}/Core/
            COMMENT "Running static code analysis with cppcheck"
            VERBATIM
        )
        
        # Optional: Run static analysis before build
        if(RUN_STATIC_ANALYSIS_ON_BUILD)
            add_dependencies(${CMAKE_PROJECT_NAME} static-analysis)
        endif()
        
        message(STATUS "Static analysis enabled with cppcheck")
    else()
        message(WARNING "cppcheck not found. Static analysis disabled.")
    endif()
endif()

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add CMSIS-DSP library
set(CMSISCORE ${CMAKE_SOURCE_DIR}/Drivers/CMSIS)
add_subdirectory(${CMAKE_SOURCE_DIR}/cmsis-dsp/Source)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/cmsis-dsp/Include)

# Add lwgps library
add_subdirectory(${CMAKE_SOURCE_DIR}/lwgps/lwgps)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lwgps/lwgps/src/include/lwgps)

# Add pq_com_format library
add_subdirectory(${CMAKE_SOURCE_DIR}/pq_com_format)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/pq_com_format/lib/include)
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
    ${CMAKE_SOURCE_DIR}/Core/devices/software/logger.c
    ${CMAKE_SOURCE_DIR}/Core/devices/battery/battery.c
    ${CMAKE_SOURCE_DIR}/Core/devices/can/can.c
    ${CMAKE_SOURCE_DIR}/Core/devices/gnss/gnss.c
    ${CMAKE_SOURCE_DIR}/Core/devices/software/valve.c
    ${CMAKE_SOURCE_DIR}/Core/devices/software/binary_packet.c
    # Add ubxlib sources
    ${UBXLIB_SOURCES}
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
    ${CMAKE_SOURCE_DIR}/Core/devices/can/
    ${CMAKE_SOURCE_DIR}/Core/devices/software/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP
    lwgps
    pq_com_format
    # Add user defined libraries
)


add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin from ELF file"
)