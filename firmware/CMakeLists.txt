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

# To build for LPC4330 (cross-compile) use the provided toolchain file:
# cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain_lpc4330.cmake
# Then build with your cross toolchain (arm-none-eabi). Ensure a linker
# script (linker.ld) suitable for LPC4330 is present at firmware/linker.ld
# and board support / startup files are configured in Core/ or Drivers/.

