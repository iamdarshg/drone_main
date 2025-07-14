cmake_minimum_required(VERSION 3.13)
project(drone_firmware C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Add source directories
add_subdirectory(Core)
add_subdirectory(Drivers)
add_subdirectory(Middleware)
add_subdirectory(App)
add_subdirectory(Config)
add_subdirectory(Utils)

# TODO: Add toolchain file for LPC4330, linker script, and FreeRTOS/LPCOpen integration
