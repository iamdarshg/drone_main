# Base Station Firmware & UI

This directory contains firmware and a user interface for a base station using the same hardware as the drone, but connected to a PC for real-time control and monitoring.

## Features
- Full S2-LPQTR RF comms with Hamming/CRC32 error correction
- Real-time telemetry and command interface
- PC connection via USB/UART
- Easy-to-use frontend UI for live control, telemetry, and configuration
- Support for controller/gamepad input
- Flight instrumentation panels
- Waypoint and mission planning

## Structure
- `firmware/` - Base station firmware for LPC4330
- `ui/` - PC frontend (cross-platform, e.g. Python/Qt or Electron)
- `examples/` - Example scripts for automation and integration

See subdirectories for details.
