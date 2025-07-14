#include "hw_sim.h"
#include "logging.h"
static int imu_ok = 1, rf_ok = 1, mem_ok = 1;
void hw_sim_init(void) { log_info("HW sim init"); }
void hw_sim_set_imu_response(int ok) { imu_ok = ok; }
void hw_sim_set_rf_response(int ok) { rf_ok = ok; }
void hw_sim_set_memory_response(int ok) { mem_ok = ok; }
// You would patch selftest_imu/rf/memory to use these in test builds
