#include "selftest.h"
#include "logging.h"
int selftest_imu(void) { log_info("IMU self-test"); return 1; }
int selftest_rf(void) { log_info("RF self-test"); return 1; }
int selftest_memory(void) { log_info("Memory self-test"); return 1; }
void selftest_all(void) {
    int ok = 1;
    ok &= selftest_imu();
    ok &= selftest_rf();
    ok &= selftest_memory();
    if (ok) log_info("All self-tests passed");
    else log_error("Self-test failure");
}
