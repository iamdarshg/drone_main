@echo off
REM build_lpc4330.bat - Build firmware for LPC4330 target (Windows)
REM Drop-in script for cross-compilation to ARM Cortex-M4

echo ==========================================
echo  Drone Firmware - LPC4330 Build Script
echo ==========================================
echo.

REM Check if we're in the right directory
if not exist "firmware\CMakeLists.txt" (
    echo ERROR: Please run this script from the project root directory
    echo Expected to find: firmware\CMakeLists.txt
    pause
    exit /b 1
)

REM Check for ARM toolchain
where arm-none-eabi-gcc >nul 2>&1
if errorlevel 1 (
    echo ERROR: ARM toolchain not found!
    echo Please install ARM GCC toolchain and add to PATH:
    echo https://developer.arm.com/downloads/-/gnu-rm
    echo.
    echo Expected tools:
    echo - arm-none-eabi-gcc
    echo - arm-none-eabi-objcopy
    echo - arm-none-eabi-size
    pause
    exit /b 1
)

echo ARM toolchain found:
arm-none-eabi-gcc --version | findstr "gcc"
echo.

REM Create build directory
if not exist "build_lpc4330" (
    echo Creating build_lpc4330 directory...
    mkdir build_lpc4330
)

cd build_lpc4330

echo Configuring build for LPC4330...
cmake -G "MinGW Makefiles" -DCMAKE_TOOLCHAIN_FILE=..\cmake\toolchain_lpc4330.cmake ..\firmware
if errorlevel 1 (
    echo ERROR: CMake configuration failed!
    echo Check that toolchain file exists: cmake\toolchain_lpc4330.cmake
    pause
    exit /b 1
)

echo.
echo Building firmware for LPC4330...
mingw32-make
if errorlevel 1 (
    echo ERROR: Build failed!
    pause
    exit /b 1
)

echo.
echo ==========================================
echo  LPC4330 Build Completed Successfully!
echo ==========================================
echo.

if exist "drone_firmware.elf" (
    echo Generated files:
    echo   drone_firmware.elf  - Debug executable
    if exist "drone_firmware.hex" echo   drone_firmware.hex  - Intel HEX format
    if exist "drone_firmware.bin" echo   drone_firmware.bin  - Binary format
    echo.
    
    echo Memory usage:
    arm-none-eabi-size drone_firmware.elf
    echo.
    
    echo Ready for programming!
    echo Use: 
    echo   - drone_firmware.hex for most programmers
    echo   - drone_firmware.bin for DFU/bootloaders
    echo   - drone_firmware.elf for debugging
) else (
    echo WARNING: drone_firmware.elf not found!
)

echo.
pause