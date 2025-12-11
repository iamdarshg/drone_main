#include "selftest.h"
#include "hw_sim.h"
#include <assert.h>
void test_selftest(void) {
    hw_sim_init();
    hw_sim_set_imu_response(1);
    hw_sim_set_rf_response(1);
    hw_sim_set_memory_response(1);
    assert(selftest_imu() == 1);
    assert(selftest_rf() == 1);
    assert(selftest_memory() == 1);
    hw_sim_set_imu_response(0);
    assert(selftest_imu() == 0);
}
int main() { test_selftest(); return 0; }
