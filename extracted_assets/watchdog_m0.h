// Enhanced M0 watchdog header with all offloaded monitoring functions
// Extends your existing watchdog_m0.h with comprehensive system monitoring

#ifndef WATCHDOG_M0_H
#define WATCHDOG_M0_H
#include <stdint.h>
#include <stdbool.h>

// Existing M0 watchdog functions
void m0_watchdog_kick(void);
void m0_watchdog_task(void);
uint32_t m0_get_status(void);

// PID/task monitoring - existing functions
void m0_monitor_pid_loop(uint32_t last_pid_tick, uint32_t now, uint32_t max_delay);
void m0_set_task_priority_flag(uint32_t flag);
void m0_clear_task_priority_flag(uint32_t flag);
uint32_t m0_get_task_priority_flags(void);
uint32_t m0_get_pid_delay_flag(void);

// Hardware error checks - existing
void m0_check_all_hardware(void);

// NEW: Battery monitoring functions - offloaded to M0
float m0_get_battery_voltage(void);
float m0_get_battery_current(void);
uint32_t m0_get_power_consumption(void);
void m0_update_battery_data(float voltage, float current);

// NEW: GPS monitoring functions - offloaded to M0
double m0_get_gps_latitude(void);
double m0_get_gps_longitude(void);
float m0_get_gps_altitude(void);
uint8_t m0_get_gps_satellites(void);
bool m0_get_gps_valid(void);
bool m0_get_home_position_set(void);
float m0_get_distance_to_home(void);
float m0_get_bearing_to_home(void);

// NEW: RF monitoring functions - offloaded to M0
uint8_t m0_get_rf_signal_strength(void);
uint16_t m0_get_rf_link_quality(void);
uint32_t m0_get_rf_tx_count(void);
uint32_t m0_get_rf_rx_count(void);
uint32_t m0_get_rf_error_count(void);
void m0_update_rf_stats(uint32_t tx_count, uint32_t rx_count, uint32_t error_count, uint8_t signal_strength);

// NEW: Temperature monitoring functions - offloaded to M0
float m0_get_temperature(uint8_t sensor_id);
bool m0_get_thermal_warning(uint8_t sensor_id);
bool m0_get_thermal_critical(uint8_t sensor_id);
void m0_update_temperature(uint8_t sensor_id, float temperature);

// NEW: Motor health monitoring functions - offloaded to M0
bool m0_get_motor_healthy(uint8_t motor_id);
uint8_t m0_get_failed_motor_mask(void);
bool m0_get_thrust_loss_detected(void);
float m0_get_motor_current(uint8_t motor_id);
void m0_update_motor_health(uint8_t motor_id, float current, uint32_t runtime);

// NEW: CPU load and task monitoring - offloaded to M0
uint16_t m0_get_cpu_load_estimate(void);
uint16_t m0_get_m4_cpu_utilization(void);
uint32_t m0_get_task_execution_time(uint8_t task_id);
uint32_t m0_get_task_deadline_misses(uint8_t task_id);
void m0_update_task_timing(uint8_t task_id, uint32_t execution_time, bool deadline_missed);
void m0_update_cpu_utilization(uint16_t m4_utilization);

// NEW: Performance counter functions - offloaded to M0
uint32_t m0_get_performance_counter(uint8_t index);
void m0_reset_performance_counters(void);
void m0_increment_performance_counter(uint8_t index);

// NEW: Telemetry functions - offloaded to M0
void m0_update_telemetry_stats(uint32_t packets_sent, uint32_t errors);
uint32_t m0_get_telemetry_packet_count(void);
uint32_t m0_get_telemetry_error_count(void);

// NEW: System health functions - computed on M0
uint8_t m0_get_system_health_score(void);
bool m0_is_system_safe_for_flight(void);

// Temperature sensor IDs for m0_get_temperature()
#define M0_TEMP_M4_CORE     0
#define M0_TEMP_IMU_KX122   1
#define M0_TEMP_IMU_LSM6DS3 2
#define M0_TEMP_IMU_LSM303C 3
#define M0_TEMP_RF_MODULE   4
#define M0_TEMP_MOTOR_ESC1  5
#define M0_TEMP_MOTOR_ESC2  6
#define M0_TEMP_AMBIENT     7

// Task IDs for task timing functions
#define M0_TASK_FLIGHT_CTRL 0
#define M0_TASK_IMU         1
#define M0_TASK_COMMAND     2
#define M0_TASK_TELEMETRY   3
#define M0_TASK_GPS         4
#define M0_TASK_THERMAL     5
#define M0_TASK_SAFETY      6
#define M0_TASK_M0_MONITOR  7

#endif