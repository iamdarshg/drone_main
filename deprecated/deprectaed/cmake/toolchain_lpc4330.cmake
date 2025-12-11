# Toolchain file for cross-compiling to LPC4330 (Cortex-M4)
# Usage: cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain_lpc4330.cmake <src>

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(NOT DEFINED CMAKE_C_COMPILER)
  set(CMAKE_C_COMPILER arm-none-eabi-gcc)
endif()
if(NOT DEFINED CMAKE_CXX_COMPILER)
  set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
endif()

set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os -g -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -T${CMAKE_SOURCE_DIR}/firmware/linker.ld")

# Provide a default sysroot / include path hints (user should adjust)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
