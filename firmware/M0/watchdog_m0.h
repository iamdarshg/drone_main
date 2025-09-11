// Enhanced watchdog_m0.h with additional telemetry and monitoring functions
// Extends the existing header with M0 offloaded functionality

#ifndef WATCHDOG_M0_H
#define WATCHDOG_M0_H
#include <stdint.h>

// Existing M0 watchdog functions
void m0_watchdog_kick(void);
void m0_watchdog_task(void);
uint32_t m0_get_status(void);

// PID/task monitoring - existing
void m0_monitor_pid_loop(uint32_t last_pid_tick, uint32_t now, uint32_t max_delay);
void m0_set_task_priority_flag(uint32_t flag);
void m0_clear_task_priority_flag(uint32_t flag);
uint32_t m0_get_task_priority_flags(void);
uint32_t m0_get_pid_delay_flag(void);

// Hardware error checks - existing
void m0_check_all_hardware(void);

// New M0 offloaded functions for telemetry support
void m0_update_telemetry_stats(uint32_t packets_sent, uint32_t errors);
uint32_t m0_get_telemetry_packet_count(void);
uint32_t m0_get_telemetry_error_count(void);
uint16_t m0_get_cpu_load_estimate(void);

// Battery monitoring functions - offloaded to M0
float m0_get_battery_voltage(void);
uint16_t m0_get_battery_adc_raw(void);

// GPS monitoring functions - offloaded to M0
uint8_t m0_get_gps_satellite_count(void);
float m0_get_gps_latitude(void);
float m0_get_gps_longitude(void);
uint8_t m0_get_gps_fix_quality(void);

// RF monitoring functions - offloaded to M0
uint8_t m0_get_rf_signal_strength(void);
uint32_t m0_get_rf_tx_count(void);
uint32_t m0_get_rf_rx_count(void);
uint32_t m0_get_rf_error_count(void);

// Performance monitoring - offloaded to M0
uint32_t m0_get_performance_counter(uint8_t index);
void m0_reset_performance_counters(void);

// Task timing monitoring - offloaded to M0
void m0_update_task_timing(uint8_t task_id, uint32_t execution_time);
uint32_t m0_get_task_timing(uint8_t task_id);

// System health summary - computed on M0
uint8_t m0_get_system_health_score(void);

#endif