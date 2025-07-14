# Drone Firmware for Dual-LPC4330 System

This project implements a redundant, dual-MCU drone controller with advanced features:

- Triple IMU fusion (LSM6DS3, KX122-1042, LSM303C)
- S2-LPQTR RF comms with Hamming code error correction
- GPS-based and pre-planned path control
- UART expansion for external modules
- Failure detection and mitigation
- Kalman filter sensor fusion
- Real-time sensor configuration
- Bus/core/RAM usage monitoring
- RF signal and error reporting
- User-friendly CLI frontend
- Robust, multi-variable PID loops

## Directory Structure

- `Core/`      : MCU startup, RTOS, LPCOpen, dual-core support
- `Drivers/`   : Hardware drivers (IMU, RF, GPS, UART, SPI, I2C, Flash, etc.)
- `Middleware/`: Hamming, Kalman, PID, monitoring, error handling
- `App/`       : Main logic, path tracing, control, comms, frontend
- `Config/`    : Board, pin, and build config
- `Utils/`     : Logging, CLI, helpers
- `Tests/`     : Unit and integration tests

## Build

- Uses LPCOpen and FreeRTOS (or similar)
- CMake/Makefile based build system

## TODO
- Implement all modules as per requirements
- See `improvements.md` for suggestions
