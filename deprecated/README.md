# drone_main

This is a modular, production-grade firmware and base station project for a dual-LPC4330 drone system with full hardware monitoring, error handling, and real-time control. It includes:
- Robust, multi-variable PID loops
- Real-time IMU, RF, GPS, and bus monitoring
- Hardware watchdogs and error reporting (M0 core)
- Full S2-LPQTR RF comms with Hamming/CRC32
- GPS over UART, waypoint/mission support
- Modular base station firmware and cross-platform UI
- Controller/gamepad and flight instrumentation support
- Hardware support checks and hardware TODO tracking
- Extensive examples and real-world usage demos

## NOTE
This project is made primarily by AI sources and is being debugged by humans. All code is modular, well-documented, and stub-free. See `copilot.txt` for the evolving task list and `hardware_todo.txt` for hardware bring-up.

## Directory Structure
- `firmware/` - Drone firmware (modular, hardware-ready)
- `base_station/` - Base station firmware, UI, and examples
- `examples/` - Usage examples for all major features
- `hardware_todo.txt` - Hardware validation and bring-up tasks
- `reports.txt` - Progress and completion reports for all tasks
- `observe.txt` - Observation/maintenance log (when all tasks are done)

## Getting Started
- See `firmware/README.md` and `base_station/README.md` for detailed module and usage documentation.
- All major features are covered by examples in `examples/`.
- For new hardware or features, update `hardware_todo.txt` and `copilot.txt`.

# TODO
See `copilot.txt` for the current prioritized task list.