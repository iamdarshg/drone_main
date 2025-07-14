#ifndef HW_SIM_H
#define HW_SIM_H
// Hardware simulation API for tests
void hw_sim_init(void);
void hw_sim_set_imu_response(int ok);
void hw_sim_set_rf_response(int ok);
void hw_sim_set_memory_response(int ok);
#endif
