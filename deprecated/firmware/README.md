# Drone Firmware: Modular Dual-LPC4330 System

## Overview
This firmware is designed for a dual-LPC4330 drone system with three IMUs (KX122, LSM6DS3/DSV, LSM303C), S2-LPQTR RF, GPS, and robust communications. It is modular, hardware-ready, and includes advanced features like error correction, sensor fusion, and real-time monitoring. The codebase is structured for clarity, maintainability, and ease of use—even for absolute beginners.

---

## Directory Structure

- **Core/**: Core system code (startup, RTOS, main loop)
- **Drivers/**: Hardware drivers (IMUs, SPI, I2C, LVDS, etc.)
- **Middleware/**: Algorithms and utilities (Kalman filter, CRC32, Hamming, self-test)
- **App/**: Main application logic and initialization
- **Config/**: Board and pin configuration, third-party setup
- **Utils/**: Logging, CLI, helper utilities
- **Tests/**: Hardware simulation, test harnesses
- **examples/**: Simple, beginner-friendly example programs

---

## Key Features & Modules

### IMU Drivers
- **KX122 (SPI):** Full register-level driver with init, read, self-test, calibration, FIFO, and free-fall detection.
- **LSM6DS3/DSV (I2C):** Register-level driver with init, read, and self-test.
- **LSM303C (I2C):** Register-level driver with init, read, and self-test.

### Error Correction
- **CRC32:** Fast, robust error checking for communications.
- **Hamming Code:** Single-bit error correction for critical data.

### Communication
- **LVDS Stub:** High-speed inter-IC comms stub (expandable for real hardware).
- **SPI/I2C Bus:** Modular, reusable bus drivers.

### Algorithms
- **Kalman Filter:** Modular 1D/3D sensor fusion for IMU data.
- **PID Loops:** (To be added) For flight control and stabilization.

### Self-Test & Simulation
- **Self-Test:** Built-in routines for IMUs, RF, and memory.
- **Hardware Simulation:** Test harnesses for running code without real hardware.

### Utilities
- **Logging:** Simple logging macros for debug/info/error output.
- **CLI:** Command-line interface for real-time configuration and monitoring.

---

## Getting Started (For Beginners)

1. **Build the firmware** using CMake. See `Config/third_party_setup.txt` for toolchain setup.
2. **Try the examples** in the `examples/` directory. Each file is a minimal, well-commented program.
3. **Read the comments** in each source file. They explain what each function does.
4. **Check the `Tests/` directory** for hardware simulation and self-test code.
5. **Use the `Utils/` directory** for logging and CLI utilities.

---

## Detailed Module Documentation

### 1. KX122 Driver (`Drivers/imu_kx122.c`/`.h`)
- **Initialization:** `imu_kx122_init()` sets up the sensor.
- **Reading Data:** `imu_kx122_read()` gets acceleration data.
- **Self-Test:** `imu_kx122_selftest()` checks if the sensor is working.
- **Calibration:** `imu_kx122_calibrate()` zeros offsets.
- **Range Setting:** `imu_kx122_set_grange()` changes measurement range.
- **FIFO Read:** `imu_kx122_read_fifo()` reads and averages FIFO samples.
- **Free-Fall:** `imu_kx122_configure_freefall()` and `imu_kx122_handle_freefall()` set up and handle free-fall detection.

### 2. LSM6DS3/DSV Driver (`Drivers/imu_lsm6ds3.c`/`.h`)
- **Initialization:** `imu_lsm6ds3_init()`
- **Reading Data:** `imu_lsm6ds3_read()`
- **Self-Test:** `imu_lsm6ds3_selftest()`

### 3. LSM303C Driver (`Drivers/imu_lsm303c.c`/`.h`)
- **Initialization:** `imu_lsm303c_init()`
- **Reading Data:** `imu_lsm303c_read()`
- **Self-Test:** `imu_lsm303c_selftest()`

### 4. CRC32 (`Middleware/crc32.c`/`.h`)
- **Usage:** `crc32_compute(data, len)` returns CRC32 checksum for a data buffer.

### 5. Hamming Code (`Middleware/hamming.c`/`.h`)
- **Usage:** `hamming_encode(data)` and `hamming_decode(encoded, &error)` for error correction.

### 6. Kalman Filter (`Middleware/kalman.c`/`.h`)
- **Usage:** `kalman_init()`, `kalman_update()`, `kalman_predict()` for sensor fusion.

### 7. LVDS Stub (`Drivers/lvds.c`/`.h`)
- **Usage:** `lvds_send(data, len)`, `lvds_receive(buf, maxlen)` for high-speed comms (stub for now).

### 8. Self-Test & Simulation (`Middleware/selftest.c`, `Tests/hw_sim.c`)
- **Self-Test:** Run `selftest_run_all()` to check all hardware modules.
- **Simulation:** Use `hw_sim.c` to run code without real hardware.

### 9. Logging & CLI (`Utils/logging.h`, `Utils/cli.h`)
- **Logging:** Use `log_info()`, `log_error()`, etc. for debug output.
- **CLI:** Add commands for runtime configuration and monitoring.

---

## Troubleshooting
- **Build errors:** Check toolchain setup in `Config/third_party_setup.txt`.
- **No sensor data:** Make sure IMU is powered and connected. Try self-test.
- **Simulation:** Use `Tests/hw_sim.c` to test without hardware.
- **More help:** Read comments in each file, or ask for help.

---

## Contributing
- Keep code modular and well-commented.
- Add new examples to `examples/`.
- Update documentation for any new features.

---

## Improvements & TODO
- Add more advanced examples (sensor fusion, PID, real-time comms).
- Expand beginner documentation as needed.
- Integrate all modules in a main application loop.
- Add more detailed API documentation in each source/header file.
- Finalize and test hardware simulation and self-test integration.

---

For any questions, see the comments in each file or contact the project maintainer.
