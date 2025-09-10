Repository dependencies and build guidance
=========================================

This document lists required tools, runtimes and recommendations to build and test
the `drone_main` project both for the LPC4330 target and for host-side VM-driven
simulation.

Host / Simulation dependencies
- Python 3.8+ and pip
- PyQt5 (used by `base_station/ui/sim_gui.py`) — see `base_station/requirements.txt`
- QEMU user-mode (qemu-arm) to run ARM user-mode ELFs under host
- GCC (host) or mingw-w64 on Windows to build host tools

ARM cross-toolchain (target build)
- arm-none-eabi-gcc / arm-none-eabi-newlib for bare-metal builds (LPC4330)
- OR arm-linux-gnueabi-gcc for producing user-mode ARM ELFs intended for qemu-arm

Build system
- CMake 3.15+ recommended for cross-platform builds

Optional / Helpful
- OpenOCD (for flashing/debugging LPC4330)
- Segger J-Link (if using J-Link hardware debugger)
- QEMU-system if you plan to try system-level emulation

Notes about LPC4330 simulation fidelity
- The LPC4330 is a dual-core Cortex-M4/M0 microcontroller with specific
  peripherals (QSPI, I2C, SPI, UART, DMA, interrupts, programmable clocking).
- Accurately simulating the LPC4330 at register/timing level requires either
  - a cycle-accurate emulator or
  - an explicit host-side shim that implements the peripheral API (registers, FIFO semantics, timing) used by the firmware. The provided `firmware/Tests/host_peripherals.*` files are a behavioral shim that enable linking and functional testing under host, but are not cycle-accurate.

Recommended next steps for higher fidelity
- Implement device-specific shims for the major peripherals (QSPI, I2C, SPI, UART) that your firmware uses. Those shims should implement the same API and return expected register values per datasheet.
- Create a `CMake` test target which builds the firmware as a user-mode ELF and links the host shims.
- Add test harnesses that exercise peripheral boundary conditions (FIFO full/empty, DMA timing, interrupt handling).
