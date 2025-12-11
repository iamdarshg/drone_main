@echo off
REM build_all.bat - Master build script for Windows
REM Drop-in complete solution for drone firmware development

echo ===========================================
echo   Drone Firmware - Master Build Script
echo ===========================================
echo.

REM Check if we're in the right directory
if not exist "firmware\CMakeLists.txt" (
    echo ERROR: Please run this script from the project root directory
    echo Expected to find: firmware\CMakeLists.txt
    echo.
    echo Current directory: %CD%
    echo.
    echo Make sure you're in the drone_main project root!
    pause
    exit /b 1
)

REM Show menu
:MENU
cls
echo ===========================================
echo   Drone Firmware - Build Menu
echo ===========================================
echo.
echo Please choose an option:
echo.
echo 1. Setup Environment          - Check and install tools
echo 2. Build Host Test            - Build for PC testing
echo 3. Build LPC4330 Target       - Build for drone hardware
echo 4. Upload Firmware GUI        - Program the drone
echo 5. Build Everything           - Complete build process
echo 6. Clean All                  - Remove build directories
echo 7. Exit
echo.
set /p choice="Enter your choice (1-7): "

if "%choice%"=="1" goto SETUP
if "%choice%"=="2" goto BUILD_HOST
if "%choice%"=="3" goto BUILD_LPC4330
if "%choice%"=="4" goto UPLOAD_GUI
if "%choice%"=="5" goto BUILD_ALL
if "%choice%"=="6" goto CLEAN_ALL
if "%choice%"=="7" goto EXIT

echo Invalid choice. Please try again.
pause
goto MENU

:SETUP
echo.
echo ===========================================
echo   Setting up development environment...
echo ===========================================
call setup_environment.bat
pause
goto MENU

:BUILD_HOST
echo.
echo ===========================================
echo   Building for host testing...
echo ===========================================
call build_host.bat
pause
goto MENU

:BUILD_LPC4330
echo.
echo ===========================================
echo   Building for LPC4330 target...
echo ===========================================
call build_lpc4330.bat
pause
goto MENU

:UPLOAD_GUI
echo.
echo ===========================================
echo   Starting firmware upload GUI...
echo ===========================================
python firmware_uploader.py
if errorlevel 1 (
    echo.
    echo Python not found or GUI failed to start
    echo Make sure Python is installed and in PATH
    pause
)
goto MENU

:BUILD_ALL
echo.
echo ===========================================
echo   Building everything...
echo ===========================================

echo Step 1: Building host version...
call build_host.bat
if errorlevel 1 (
    echo Host build failed!
    pause
    goto MENU
)

echo.
echo Step 2: Building LPC4330 version...
call build_lpc4330.bat
if errorlevel 1 (
    echo LPC4330 build failed!
    pause
    goto MENU
)

echo.
echo ===========================================
echo   Complete Build Summary
echo ===========================================
echo.

if exist "build_host\drone_firmware_test.exe" (
    echo ✓ Host build successful
    echo   File: build_host\drone_firmware_test.exe
) else (
    echo X Host build missing
)

if exist "build_lpc4330\drone_firmware.elf" (
    echo ✓ LPC4330 build successful
    echo   Files: build_lpc4330\drone_firmware.*
    echo.
    echo Generated firmware files:
    if exist "build_lpc4330\drone_firmware.hex" echo   - drone_firmware.hex  (for programmers)
    if exist "build_lpc4330\drone_firmware.bin" echo   - drone_firmware.bin  (for bootloaders)
    echo   - drone_firmware.elf  (for debugging)
    
    echo.
    cd build_lpc4330
    if exist "drone_firmware.elf" (
        echo Memory usage:
        arm-none-eabi-size drone_firmware.elf 2>nul
        if errorlevel 1 echo   [Memory info not available - arm-none-eabi-size not found]
    )
    cd ..
) else (
    echo X LPC4330 build missing
)

echo.
echo Next steps:
echo   1. Test host version: build_host\drone_firmware_test.exe
echo   2. Program drone: python firmware_uploader.py
echo   3. Or use: program_lpc4330.bat

echo.
set /p choice="Open firmware uploader GUI now? (y/n): "
if /i "%choice%"=="y" (
    python firmware_uploader.py
)

pause
goto MENU

:CLEAN_ALL
echo.
echo ===========================================
echo   Cleaning all build directories...
echo ===========================================

echo Removing build directories...
if exist "build_host" (
    rmdir /s /q build_host
    echo ✓ Removed build_host
)

if exist "build_lpc4330" (
    rmdir /s /q build_lpc4330
    echo ✓ Removed build_lpc4330
)

REM Clean temporary files
if exist "openocd_lpc4330.cfg" del openocd_lpc4330.cfg
if exist "temp_upload.cfg" del temp_upload.cfg
if exist "program_commands.cfg" del program_commands.cfg

echo.
echo All build directories cleaned!
echo.
pause
goto MENU

:EXIT
echo.
echo Thank you for using the drone firmware build system!
echo.
exit /b 0