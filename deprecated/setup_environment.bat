@echo off
REM setup_environment.bat - Setup development environment (Windows)
REM Drop-in script to check and install required tools

echo ==========================================
echo  Drone Firmware - Environment Setup
echo ==========================================
echo.

set TOOLS_MISSING=0

echo Checking development environment...
echo.

REM Check CMake
echo [1/6] Checking CMake...
where cmake >nul 2>&1
if errorlevel 1 (
    echo   X CMake not found
    echo     Download from: https://cmake.org/download/
    set TOOLS_MISSING=1
) else (
    cmake --version | findstr "cmake"
    echo   ✓ CMake found
)
echo.

REM Check MinGW
echo [2/6] Checking MinGW...
where mingw32-make >nul 2>&1
if errorlevel 1 (
    echo   X MinGW not found
    echo     Download from: https://www.mingw-w64.org/
    echo     Or install via MSYS2: pacman -S mingw-w64-x86_64-toolchain
    set TOOLS_MISSING=1
) else (
    mingw32-make --version | findstr "GNU Make"
    echo   ✓ MinGW found
)
echo.

REM Check ARM GCC Toolchain
echo [3/6] Checking ARM GCC Toolchain...
where arm-none-eabi-gcc >nul 2>&1
if errorlevel 1 (
    echo   X ARM GCC Toolchain not found
    echo     Download from: https://developer.arm.com/downloads/-/gnu-rm
    echo     Required for LPC4330 cross-compilation
    set TOOLS_MISSING=1
) else (
    arm-none-eabi-gcc --version | findstr "gcc"
    echo   ✓ ARM GCC Toolchain found
)
echo.

REM Check Python (for GUI tools)
echo [4/6] Checking Python...
where python >nul 2>&1
if errorlevel 1 (
    echo   X Python not found
    echo     Download from: https://www.python.org/downloads/
    echo     Required for firmware upload GUI
    set TOOLS_MISSING=1
) else (
    python --version
    echo   ✓ Python found
)
echo.

REM Check OpenOCD (for programming)
echo [5/6] Checking OpenOCD...
where openocd >nul 2>&1
if errorlevel 1 (
    echo   X OpenOCD not found (optional)
    echo     Download from: https://openocd.org/pages/getting-openocd.html
    echo     Required for JTAG/SWD programming
    echo   - Not critical for building firmware
) else (
    openocd --version 2>&1 | findstr "Open On-Chip Debugger"
    echo   ✓ OpenOCD found
)
echo.

REM Check Git
echo [6/6] Checking Git...
where git >nul 2>&1
if errorlevel 1 (
    echo   X Git not found
    echo     Download from: https://git-scm.com/downloads
    echo     Required for version control
) else (
    git --version
    echo   ✓ Git found
)
echo.

REM Install Python packages if Python is available
where python >nul 2>&1
if not errorlevel 1 (
    echo Installing required Python packages...
    python -m pip install --upgrade pip
    python -m pip install tkinter pyserial requests tqdm
    echo   ✓ Python packages installed
    echo.
)

REM Summary
echo ==========================================
echo  Environment Check Summary
echo ==========================================
echo.

if %TOOLS_MISSING%==0 (
    echo ✓ All required tools are installed!
    echo.
    echo Ready to build firmware:
    echo   - Run build_host.bat for testing
    echo   - Run build_lpc4330.bat for deployment
    echo   - Run firmware_uploader.py for programming
    echo.
    echo Quick start:
    echo   1. build_host.bat       - Test on PC
    echo   2. build_lpc4330.bat    - Build for drone
    echo   3. python firmware_uploader.py - Program drone
) else (
    echo X Some tools are missing - please install them first
    echo.
    echo Required tools:
    echo   - CMake (build system)
    echo   - MinGW (compiler for host)
    echo   - ARM GCC (compiler for LPC4330)
    echo   - Python (for upload GUI)
    echo.
    echo Optional tools:
    echo   - OpenOCD (for JTAG programming)
    echo   - Git (version control)
)

echo.
pause