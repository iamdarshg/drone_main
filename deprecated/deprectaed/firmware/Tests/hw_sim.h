#ifndef HW_SIM_H
#define HW_SIM_H
// Hardware simulation API for tests
void hw_sim_init(void);
void hw_sim_set_imu_response(int ok);
void hw_sim_set_rf_response(int ok);
void hw_sim_set_memory_response(int ok);
/* Set full path to ARM user-mode emulator (e.g. qemu-arm). Optional. */
void hw_sim_set_emulator_path(const char *path);
/* Set timeout for emulator run (seconds). */
void hw_sim_set_timeout(int seconds);
/* Run compiled ARM ELF under VM emulator; returns 0 on success. */
int hw_sim_run_vm_tests(const char *elf_path);
#endif
