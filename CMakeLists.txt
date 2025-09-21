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
set(CMAKE_PROJECT_NAME StartM3508)

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
add_executable(${CMAKE_PROJECT_NAME}
        Bsp/Src/bsp_can.cpp
        Bsp/Inc/bsp_can.h
        PID/Src/speed_pid.cpp
        PID/Inc/speed_pid.h
        Debug-Vars/debug_vars.cpp
        Debug-Vars/debug_vars.h
        Remote/Src/dt7_remote.cpp
        Remote/Inc/dt7_remote.h
        Remote/Src/dbus.cpp
        Remote/Inc/dbus.h
        Task/Src/motorTask.cpp
        Task/Inc/motorTask.h
        Task/Inc/TaskBase.h
        Task/Src/chassisTask.cpp
        Task/Inc/chassisTask.h
        Chassis/Src/mecanum.cpp
        Chassis/Inc/mecanum.h
        PID/Src/angle_pid.cpp
        PID/Inc/angle_pid.h
        PID/Inc/PidBase.h
        Bsp/magictools.h
        PID/Src/CascadePID.cpp
        PID/Inc/CascadePId.h
)

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
        Bsp
        Bsp/Inc
        Bsp/Src
        PID/Inc
        PID/Src
        Debug-Vars
        Remote/Inc
        Remote/Src
        Task/Inc
        Task/Src
        Chassis/Inc
        Chassis/Src
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
