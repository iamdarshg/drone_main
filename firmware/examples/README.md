# Firmware Examples for Beginners

This directory contains simple, step-by-step example programs for each major module in the drone firmware. These are designed for absolute beginners and demonstrate how to use the IMU drivers, error correction, Kalman filter, and more. Each example is self-contained and includes comments explaining every step.

## Example List

- `imu_kx122_example.c`: How to initialize and read from the KX122 accelerometer.
- `lsm6ds3_example.c`: How to use the LSM6DS3 IMU driver.
- `lsm303c_example.c`: How to use the LSM303C IMU driver.
- `kalman_example.c`: How to use the Kalman filter for sensor fusion.
- `crc32_example.c`: How to use CRC32 for error checking.
- `hamming_example.c`: How to use Hamming code for error correction.
- `lvds_example.c`: How to use the LVDS communication stub.

## How to Run Examples

1. **Build the firmware** using the provided CMake setup. See the main `README.md` for build instructions.
2. **Open any example file** in this directory. Each file is a complete, minimal program.
3. **Follow the comments** in each file. They explain what each line does.
4. **Flash to your board** or run in simulation (see `Tests/hw_sim.c` for hardware simulation).

## Tips for Beginners

- Read the comments in each example carefully. They explain every step.
- If you get stuck, check the main `README.md` for troubleshooting.
- You do **not** need to understand the full firmware to use these examples.
- Each example is independent. Start with the IMU examples, then try error correction and Kalman filter.

---

For more help, see the main documentation or ask for help in your team or online forums.
