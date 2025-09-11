#!/bin/bash
# build_all.sh - Complete build script for drone firmware (Linux/macOS)
# Drop-in script for both host testing and LPC4330 deployment

set -e  # Exit on any error

echo "=========================================="
echo "  Drone Firmware - Complete Build Script"
echo "=========================================="
echo

# Check if we're in the right directory
if [ ! -f "firmware/CMakeLists.txt" ]; then
    echo "ERROR: Please run this script from the project root directory"
    echo "Expected to find: firmware/CMakeLists.txt"
    exit 1
fi

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

echo "Checking build environment..."

# Check CMake
if ! command_exists cmake; then
    echo "ERROR: CMake not found"
    echo "Install with: sudo apt-get install cmake  (Ubuntu/Debian)"
    echo "         or: brew install cmake        (macOS)"
    exit 1
fi

# Check GCC
if ! command_exists gcc; then
    echo "ERROR: GCC not found"
    echo "Install with: sudo apt-get install build-essential  (Ubuntu/Debian)"
    echo "         or: xcode-select --install                (macOS)"
    exit 1
fi

echo "✓ Build environment OK"
echo

# Build for host testing
echo "=========================================="
echo "1. Building for Host Testing"
echo "=========================================="

mkdir -p build_host
cd build_host

echo "Configuring host build..."
cmake ../firmware
echo "Building host version..."
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

if [ -f "drone_firmware_test" ]; then
    echo "✓ Host build successful: build_host/drone_firmware_test"
    
    # Ask if user wants to run test
    echo
    read -p "Run host test now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Running host test..."
        ./drone_firmware_test
    fi
else
    echo "ERROR: Host build failed"
    exit 1
fi

cd ..

# Build for LPC4330 if ARM toolchain available
echo
echo "=========================================="
echo "2. Building for LPC4330 Target"
echo "=========================================="

if command_exists arm-none-eabi-gcc; then
    echo "ARM toolchain found:"
    arm-none-eabi-gcc --version | head -n 1
    echo
    
    mkdir -p build_lpc4330
    cd build_lpc4330
    
    echo "Configuring LPC4330 build..."
    cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain_lpc4330.cmake ../firmware
    
    echo "Building LPC4330 version..."
    make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
    
    if [ -f "drone_firmware.elf" ]; then
        echo "✓ LPC4330 build successful"
        echo
        echo "Generated files:"
        ls -la drone_firmware.*
        echo
        echo "Memory usage:"
        arm-none-eabi-size drone_firmware.elf
        echo
        echo "Ready for programming!"
    else
        echo "ERROR: LPC4330 build failed"
        exit 1
    fi
    
    cd ..
    
else
    echo "ARM toolchain not found - skipping LPC4330 build"
    echo "Install with:"
    echo "  Ubuntu/Debian: sudo apt-get install gcc-arm-none-eabi"
    echo "  macOS: brew install --cask gcc-arm-embedded"
    echo "  Or download from: https://developer.arm.com/downloads/-/gnu-rm"
fi

echo
echo "=========================================="
echo "  Build Summary"
echo "=========================================="
echo

if [ -f "build_host/drone_firmware_test" ]; then
    echo "✓ Host build: build_host/drone_firmware_test"
fi

if [ -f "build_lpc4330/drone_firmware.elf" ]; then
    echo "✓ LPC4330 build: build_lpc4330/drone_firmware.elf"
    echo "  Programming files:"
    echo "    - drone_firmware.hex (for most programmers)"
    echo "    - drone_firmware.bin (for DFU/bootloaders)"
fi

echo
echo "Next steps:"
echo "  1. Test: ./build_host/drone_firmware_test"
if [ -f "build_lpc4330/drone_firmware.elf" ]; then
    echo "  2. Program: python3 firmware_uploader.py"
    echo "         or: ./program_lpc4330.sh"
fi

echo
echo "Build completed successfully! 🚁"