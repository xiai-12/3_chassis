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

# Set the project name
set(CMAKE_PROJECT_NAME 3_chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Hardware/Usart/Serial.c
        Core/Hardware/Dji/m3508.c
        Core/Hardware/Dji/m3508.h
        Core/Hardware/PID/pid.c
        Core/Application/src/serial_cmd.c
        Core/Application/src/motor_control.c
        Core/Application/inc/motor_control.h
        Core/Application/src/motion_plane.c
        Core/Application/inc/motion_plane.h
        Core/Hardware/Key/key.c
        Core/Hardware/Key/key.h
        Core/Hardware/Chassis/chassis.c
        Core/Hardware/Nrf/Control_slave.hpp
        Core/Hardware/Nrf/Control_slave.h
        Core/Hardware/Nrf/Control_slave.cpp
        Core/Hardware/Nrf/nrf24.c
        Core/Application/src/nrf_cmd.c
        Core/Application/inc/nrf_cmd.h
        Core/Hardware/Vision/connect.c
        Core/Hardware/Vision/connect.h
)

# print float
target_link_options(${PROJECT_NAME} PRIVATE -u _printf_float)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/Application/inc
        Core/Hardware
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
