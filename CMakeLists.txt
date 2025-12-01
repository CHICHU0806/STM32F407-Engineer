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

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
        Communication/Src/usart_dma.cpp
        Communication/Inc/usart_dma.h
        Communication/Src/dbus.cpp
        Communication/Inc/dbus.h
        Task/Src/motorTask.cpp
        Task/Inc/motorTask.h
        Task/TaskBase.h
        Task/Src/mecanumTask.cpp
        Task/Inc/mecanumTask.h
        Chassis/Src/mecanum.cpp
        Chassis/Inc/mecanum.h
        PID/Src/angle_pid.cpp
        PID/Inc/angle_pid.h
        PID/PidBase.h
        Bsp/magictools.h
        Task/Src/measureHeroFourSpeedTask.cpp
        Task/Inc/measureHeroFourSpeedTask.h
        Chassis/Src/omniwheel.cpp
        Chassis/Inc/omniwheel.h
        Task/Src/omniwheelTask.cpp
        Task/Inc/omniwheelTask.h
        Task/Src/keyTask.cpp
        Task/Inc/keyTask.h
        Algorithm/Src/CRC16.cpp
        Algorithm/Inc/CRC16.h
        Algorithm/Src/CRC8.cpp
        Algorithm/Inc/CRC8.h
        Message_Bus/message_bus.cpp
        Message_Bus/message_bus.h
        Chassis/Src/swerve.cpp
        Chassis/Inc/swerve.h
        Task/Src/swerveTask.cpp
        Task/Inc/swerveTask.h
        Task/Src/targetCarTask.cpp
        Task/Inc/targetCarTask.h
        Communication/Src/usart_protocol.cpp
        Communication/Inc/usart_protocol.h
        Communication/Src/usart_decode.cpp
        Communication/Inc/usart_decode.h
        Task/Src/masterBoardTask.cpp
        Task/Inc/masterBoardTask.h
        Task/Src/measureInfantryTwoSpeedTask.cpp
        Task/Inc/measureInfantryTwoSpeedTask.h
        Task/Src/slaveBoardTask.cpp
        Task/Inc/slaveBoardTask.h
        BMI088/Src/BMI088driver.c
        BMI088/Inc/BMI088driver.h
        BMI088/Src/BMI088Middleware.c
        BMI088/Inc/BMI088Middleware.h
        BMI088/BMI088config.h
        Bsp/Src/bsp_dwt.cpp
        Bsp/Inc/bsp_dwt.h
        IST8310/Src/ist8310driver_middleware.c
        IST8310/Inc/ist8310driver.h
        IST8310/Src/ist8310driver.c
        IST8310/Inc/ist8310driver_middleware.h
        PID/Src/temp_pid.cpp
        PID/Inc/temp_pid.h
        Bsp/Src/bsp_spi.c
        Bsp/Inc/bsp_spi.h
        Algorithm/Src/MahonyAHRS.c
        Algorithm/Inc/MahonyAHRS.h
        BMI088/Src/ImuTempControl.cpp
        BMI088/Src/BMI088.cpp
        BMI088/Inc/BMI088.h
        Task/Src/imuTask.cpp
        Task/Inc/imuTask.h
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
        PID
        Debug-Vars
        Communication/Inc
        Communication/Src
        Task
        Task/Inc
        Task/Src
        Chassis/Inc
        Chassis/Src
        Algorithm/Inc
        Algorithm/Src
        Message_Bus
        BMI088/Inc
        BMI088/Src
        BMI088
        IST8310/Inc
        IST8310/Src
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
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
