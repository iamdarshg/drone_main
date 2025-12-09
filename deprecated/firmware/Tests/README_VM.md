VM-driven hardware test runner
=================================

This directory contains a small host-side test runner that uses the
`hw_sim` VM interface to run compiled ARM test ELF images under an
ARM user-mode emulator (for example `qemu-arm`).

Quick steps (Windows PowerShell):

1. Install QEMU with user-mode support (qemu-arm). Ensure `qemu-arm.exe` is on PATH.
2. Build the test runner with your host toolchain. Using mingw-w64 (gcc):

```powershell
gcc -I. -o run_vm_tests run_vm_tests.c hw_sim.c logging.c
```

3. Build the firmware/tests for ARM as an ELF. Example toolchain command (arm-none-eabi-gcc):

```powershell
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -g -O0 -o test_arm.elf <your_test_sources> -specs=nosys.specs
```

4. Run the host runner and point it to the ARM ELF. The host runner will start the emulator,
   capture stdout, and look for lines beginning with `TEST:`. Example:

```powershell
.\run_vm_tests.exe .\test_arm.elf
```

Notes:
- The ARM ELF should be built for Linux/ELF ARM user-mode if you plan to run it directly
  under qemu-arm user-mode. If you have a bare-metal firmware image, you'll need to add a
  small Linux-compatible harness or build a special test ELF that links against the host
  runtime. The runner expects the ELF to print `TEST: <name> PASS|FAIL` lines.
- This runner is intended for host-side integration tests only and should not be linked
  into target firmware builds that run on the LPC4330.
