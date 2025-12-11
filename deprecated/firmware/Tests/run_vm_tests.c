/*
 * run_vm_tests.c
 *
 * Simple host-side test runner that invokes the hw_sim VM test runner to
 * execute a compiled ARM ELF under an ARM user-mode emulator (qemu-arm).
 *
 * Build (host):
 *  - On Windows: compile with your host toolchain (e.g., MSVC or mingw)
 *  - On Linux/macOS: gcc -o run_vm_tests run_vm_tests.c hw_sim.c logging.c -I.
 *
 * Usage:
 *  run_vm_tests <path-to-arm-elf>
 */

#include "hw_sim.h"
#include "logging.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <path-to-arm-elf>\n", argv[0]);
        return 2;
    }

    const char *elf = argv[1];

    hw_sim_init();
    /* Optional: override emulator binary if not on PATH */
    /* hw_sim_set_emulator_path("C:\\path\\to\\qemu-arm.exe"); */

    int rc = hw_sim_run_vm_tests(elf);
    if (rc == 0) {
        printf("All VM tests passed.\n");
        return 0;
    }

    printf("VM tests failed (code %d)\n", rc);
    return rc;
}
