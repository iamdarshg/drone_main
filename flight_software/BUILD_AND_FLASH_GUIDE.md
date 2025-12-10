# Build and Flash Guide

This guide provides instructions on how to build the flight software and flash it to the target hardware.

## Prerequisites

*   An ARM cross-compilation toolchain installed on your system.
*   The F' framework installed and configured.
*   The project has been cloned to your local machine.

## Build

1.  **Navigate to the `flight_software` directory.**
    ```
    cd flight_software
    ```

2.  **Generate the build files.**
    ```
    fprime-util generate
    ```

3.  **Build the flight software.**
    ```
    fprime-util build
    ```

## Flash

1.  **Connect the target hardware to your computer.**

2.  **Identify the serial port of the target hardware.**

3.  **Use a flashing tool to upload the firmware to the target.**
    *   The firmware will be located in the `build-fprime-automatic-native/bin` directory.
    *   The specific flashing tool will depend on the target hardware.

This guide will be updated with more specific instructions as the hardware is finalized.
