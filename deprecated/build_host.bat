@echo off
REM build_host.bat - Build firmware for host testing (Windows)
REM Drop-in script for firmware development and testing

echo ========================================
echo  Drone Firmware - Host Build Script
echo ========================================
echo.

REM Check if we're in the right directory
if not exist "firmware\CMakeLists.txt" (
    echo ERROR: Please run this script from the project root directory
    echo Expected to find: firmware\CMakeLists.txt
    pause
    exit /b 1
)

REM Create build directory
if not exist "build_host" (
    echo Creating build_host directory...
    mkdir build_host
)

cd build_host

echo Configuring build for host testing...
cmake -G "MinGW Makefiles" ..\firmware
if errorlevel 1 (
    echo ERROR: CMake configuration failed!
    echo Make sure you have CMake and MinGW installed
    pause
    exit /b 1
)

echo.
echo Building firmware for host...
mingw32-make
if errorlevel 1 (
    echo ERROR: Build failed!
    pause
    exit /b 1
)

echo.
echo ========================================
echo  Host Build Completed Successfully!
echo ========================================
echo.
echo Executable: build_host\drone_firmware_test.exe
echo.
echo To run the test:
echo   cd build_host
echo   drone_firmware_test.exe
echo.

REM Ask if user wants to run the test
set /p choice="Run the test now? (y/n): "
if /i "%choice%"=="y" (
    echo.
    echo Running host test...
    drone_firmware_test.exe
)

pause