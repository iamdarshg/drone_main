# M0 Core Firmware (LPC4330)

This directory contains code to be compiled for the M0 core. The M0 is responsible for hardware watchdog, monitoring, self-test, and error handling functions, while the M4 core runs the main application.

## Features
- Hardware watchdog timer (resets or alerts M4 if main loop stalls)
- Monitoring of system status flags
- Hardware self-test and error flag management
- Simple API for M4 to query M0 status and errors

## Usage
- Build this code with the M0 toolchain and linker script.
- Link with the main firmware as a separate binary or section.
- Call `m0_watchdog_task()` and `m0_selftest_run()` in a loop on the M0 core.
- M4 can query status and errors via shared memory or IPC.

See `watchdog_m0.c`, `watchdog_m0.h`, `selftest_m0.c`, and `selftest_m0.h` for details.
