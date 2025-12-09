@echo off
REM install_requirements.bat - Automated tool installation for Windows
REM Drop-in script to install all required development tools

echo ===========================================
echo   Drone Firmware - Requirements Installer
echo ===========================================
echo.
echo This script will help install required development tools:
echo   - CMake (build system)
echo   - MinGW-w64 (C compiler)
echo   - ARM GCC Toolchain (cross-compiler)
echo   - Python + packages (for GUI tools)
echo   - OpenOCD (optional - for programming)
echo.

set /p choice="Continue with installation? (y/n): "
if /i not "%choice%"=="y" exit /b 0

echo.
echo ===========================================
echo   Installing Development Tools
echo ===========================================

REM Check if winget is available (Windows Package Manager)
where winget >nul 2>&1
if not errorlevel 1 (
    echo Using winget for installations 
    
    echo [1/5] Installing CMake 
    winget install Kitware.CMake --accept-source-agreements --accept-package-agreements
    
    echo [2/5] Installing Python 
    winget install Python.Python.3.11 --accept-source-agreements --accept-package-agreements
    
    echo [3/5] Installing Git 
    winget install Git.Git --accept-source-agreements --accept-package-agreements
    
    echo [4/5] Installing MSYS2 (for MinGW) 
    winget install MSYS2.MSYS2 --accept-source-agreements --accept-package-agreements
    
    echo [5/5] Installing OpenOCD 
    winget install OpenOCD.OpenOCD --accept-source-agreements --accept-package-agreements
    
    echo.
    echo ✓ Package installation complete!
    echo.
    
) else (
    echo winget not available. Please install manually:
    echo.
    echo Required downloads:
    echo   1. CMake: https://cmake.org/download/
    echo   2. Python: https://www.python.org/downloads/
    echo   3. Git: https://git-scm.com/downloads
    echo   4. MSYS2: https://www.msys2.org/
    echo   5. OpenOCD: https://openocd.org/pages/getting-openocd.html
    echo.
    pause
)

REM Install MSYS2 packages for MinGW and ARM toolchain
echo ===========================================
echo   Installing MinGW and ARM Toolchain
echo ===========================================
echo.
echo Opening MSYS2 for toolchain installation 
echo Please run these commands in the MSYS2 terminal:
echo.
echo   pacman -Syu
echo   pacman -S mingw-w64-x86_64-toolchain
echo   pacman -S mingw-w64-x86_64-cmake
echo   pacman -S mingw-w64-x86_64-arm-none-eabi-toolchain
echo.
echo After installation, add to Windows PATH:
echo   C:\msys64\mingw64\bin
echo   C:\msys64\usr\bin
echo.

REM Try to open MSYS2
if exist "C:\msys64\msys2.exe" (
    start "MSYS2" "C:\msys64\msys2.exe"
) else (
    echo MSYS2 not found at C:\msys64\
    echo Please install MSYS2 from: https://www.msys2.org/
)

echo.
set /p choice="Press Enter after installing MSYS2 packages "

REM Install Python packages
echo ===========================================
echo   Installing Python Packages
echo ===========================================
echo.

where python >nul 2>&1
if not errorlevel 1 (
    echo Installing required Python packages 
    python -m pip install --upgrade pip
    python -m pip install tkinter pyserial requests tqdm
    
    if errorlevel 1 (
        echo Warning: Some Python packages may have failed to install
    ) else (
        echo ✓ Python packages installed successfully
    )
) else (
    echo Python not found in PATH
    echo Please install Python and add to PATH
)

echo.
echo ===========================================
echo   PATH Configuration
echo ===========================================
echo.
echo Please add these directories to your Windows PATH:
echo.
echo From MSYS2 installation:
echo   C:\msys64\mingw64\bin
echo   C:\msys64\usr\bin
echo.
echo From CMake installation:
echo   C:\Program Files\CMake\bin
echo.
echo From Python installation:
echo   C:\Python311\
echo   C:\Python311\Scripts\
echo.
echo To add to PATH:
echo   1. Press Win+R, type 'sysdm.cpl', press Enter
echo   2. Click 'Environment Variables'
echo   3. Select 'Path' in System Variables
echo   4. Click 'Edit' and add the paths above
echo   5. Click OK and restart your command prompt
echo.

echo ===========================================
echo   Installation Complete!
echo ===========================================
echo.
echo Installed tools:
echo   ✓ CMake (build system)
echo   ✓ MinGW-w64 (C compiler)  
echo   ✓ ARM GCC (cross-compiler)
echo   ✓ Python + packages (GUI tools)
echo   ✓ OpenOCD (programmer)
echo   ✓ Git (version control)
echo.
echo Next steps:
echo   1. Restart your command prompt
echo   2. Run: setup_environment.bat (to verify installation)
echo   3. Run: build_all.bat (to start building)
echo.
echo If you encounter issues, check that all tools are in your PATH.
echo.
pause