// Enhanced M0 watchdog implementation with all thermal and performance monitoring
// This file extends your existing watchdog_m0.c with comprehensive monitoring

#include "watchdog_m0.h"
#include "selftest_m0.h"
#include "bus_monitor.h"
#include "imu_monitor.h" 
#include "rf_monitor.h"
#include "ram_monitor.h"
#include "core_monitor.h"
#include "gps_uart.h"
#include <stdint.h>

// Existing M0 state variables
volatile uint32_t m0_watchdog_counter = 0;
volatile uint32_t m0_status_flags = 0;
volatile uint32_t m0_pid_delay_flag = 0;
volatile uint32_t m0_task_priority_flags = 0;

// NEW: Enhanced monitoring variables for all offloaded functions
volatile uint32_t m0_telemetry_packet_count = 0;
volatile uint32_t m0_telemetry_error_count = 0;
volatile uint16_t m0_cpu_load_estimate = 10;
volatile uint32_t m0_performance_counters[16] = {0}; // Expanded to 16 counters

// NEW: Battery and power monitoring - offloaded to M0
volatile float m0_battery_voltage = 12.6f;
volatile uint16_t m0_battery_adc_raw = 0;
volatile float m0_battery_current = 0.0f;
volatile uint32_t m0_power_consumption = 0; // mW

// NEW: GPS monitoring and RTH support - offloaded to M0
volatile uint8_t m0_gps_satellite_count = 0;
volatile double m0_gps_latitude = 0.0;
volatile double m0_gps_longitude = 0.0;
volatile float m0_gps_altitude = 0.0f;
volatile uint8_t m0_gps_fix_quality = 0;
volatile bool m0_gps_valid = false;
volatile double m0_home_latitude = 0.0;
volatile double m0_home_longitude = 0.0;
volatile bool m0_home_position_set = false;
volatile float m0_distance_to_home = 0.0f;
volatile float m0_bearing_to_home = 0.0f;

// NEW: RF monitoring and link quality - offloaded to M0
volatile uint8_t m0_rf_signal_strength = 85;
volatile uint32_t m0_rf_tx_count = 0;
volatile uint32_t m0_rf_rx_count = 0;
volatile uint32_t m0_rf_error_count = 0;
volatile uint16_t m0_rf_link_quality = 100;
volatile uint32_t m0_rf_last_packet_time = 0;

// NEW: Temperature monitoring - offloaded to M0
volatile float m0_temperatures[8] = {25.0f, 25.0f, 25.0f, 25.0f, 25.0f, 25.0f, 25.0f, 25.0f};
volatile bool m0_thermal_warning_flags[8] = {false};
volatile bool m0_thermal_critical_flags[8] = {false};
volatile float m0_max_safe_temp = 85.0f;
volatile float m0_warning_temp = 75.0f;

// NEW: Motor health monitoring - offloaded to M0
volatile bool m0_motor_healthy[4] = {true, true, true, true};
volatile float m0_motor_current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile uint32_t m0_motor_error_count[4] = {0, 0, 0, 0};
volatile uint8_t m0_failed_motor_mask = 0;
volatile bool m0_thrust_loss_detected = false;
volatile uint32_t m0_motor_runtime[4] = {0, 0, 0, 0}; // Runtime in seconds

// NEW: Task timing and CPU load monitoring - offloaded to M0
volatile uint32_t m0_task_execution_times[8] = {0}; // Track 8 different tasks
volatile uint32_t m0_task_deadline_misses[8] = {0};
volatile uint16_t m0_m4_cpu_utilization = 25;
volatile uint32_t m0_context_switches = 0;
volatile uint32_t m0_interrupt_count = 0;

// Existing M0 functions with enhancements
void m0_watchdog_kick(void) {
    m0_watchdog_counter = 0;
}

void m0_watchdog_task(void) {
    // Enhanced watchdog with performance tracking
    m0_watchdog_counter++;
    
    // Dynamic CPU load estimation based on activity
    uint32_t activity_level = (m0_rf_tx_count + m0_rf_rx_count + m0_context_switches) % 100;
    m0_cpu_load_estimate = 10 + activity_level / 5; // 10-30% range
    
    if (m0_watchdog_counter > 1000000) {
        m0_status_flags |= 0x01; // Timeout flag
        // Could trigger M4 reset or alert
    }
    
    m0_performance_counters[0]++; // Watchdog iterations
}

void m0_monitor_pid_loop(uint32_t last_pid_tick, uint32_t now, uint32_t max_delay) {
    uint32_t delay = now - last_pid_tick;
    
    if (delay > max_delay) {
        m0_pid_delay_flag = 1;
        m0_performance_counters[1]++; // PID delay events
    } else {
        m0_pid_delay_flag = 0;
    }
    
    // Store timing for performance analysis
    m0_task_execution_times[0] = delay; // Flight control task timing
}

void m0_set_task_priority_flag(uint32_t flag) {
    m0_task_priority_flags |= flag;
}

void m0_clear_task_priority_flag(uint32_t flag) {
    m0_task_priority_flags &= ~flag;
}

uint32_t m0_get_task_priority_flags(void) {
    return m0_task_priority_flags;
}

uint32_t m0_get_pid_delay_flag(void) {
    return m0_pid_delay_flag;
}

uint32_t m0_get_status(void) {
    return m0_status_flags;
}

// Enhanced hardware monitoring with comprehensive safety checks
void m0_check_all_hardware(void) {
    // Existing bus monitoring
    if (bus_monitor_get_spi_bytes() > 1000000 || bus_monitor_get_i2c_bytes() > 1000000) {
        m0_error_set(0x01);
        bus_monitor_reset();
        m0_performance_counters[2]++;
    }
    
    // IMU health monitoring
    imu_monitor_check_all();
    if (imu_monitor_get_errors()) {
        m0_error_set(0x02);
        m0_performance_counters[3]++;
    }
    
    // NEW: Comprehensive temperature monitoring
    for (int i = 0; i < 8; i++) {
        if (m0_temperatures[i] > m0_warning_temp) {
            m0_thermal_warning_flags[i] = true;
        }
        if (m0_temperatures[i] > m0_max_safe_temp) {
            m0_thermal_critical_flags[i] = true;
            m0_error_set(0x200 + i); // Thermal error per sensor
        }
    }
    
    // NEW: Motor health monitoring
    for (int i = 0; i < 4; i++) {
        // Check for motor overcurrent
        if (m0_motor_current[i] > 10.0f) { // 10A limit
            m0_motor_healthy[i] = false;
            m0_failed_motor_mask |= (1 << i);
            m0_error_set(0x100 + i); // Motor error per motor
        }
        
        // Check for motor undercurrent (possible failure)
        if (m0_motor_current[i] < 0.1f && m0_motor_runtime[i] > 60) { // Running for >1min
            m0_motor_error_count[i]++;
            if (m0_motor_error_count[i] > 10) {
                m0_motor_healthy[i] = false;
                m0_failed_motor_mask |= (1 << i);
            }
        }
    }
    
    // NEW: RF link monitoring with detailed statistics
    uint32_t current_time = 0; // Would get from timer
    if (current_time - m0_rf_last_packet_time > 1000) { // 1 second timeout
        m0_rf_link_quality = 0;
        m0_error_set(0x04); // RF timeout
    } else {
        // Calculate link quality based on packet success rate
        if (m0_rf_tx_count > 0) {
            m0_rf_link_quality = ((m0_rf_tx_count - m0_rf_error_count) * 100) / m0_rf_tx_count;
        }
    }
    
    // NEW: GPS monitoring with RTH support
    gps_data_t *gps_data = gps_uart_get_data();
    if (gps_data) {
        m0_gps_satellite_count = gps_data->satellites;
        m0_gps_latitude = gps_data->latitude;
        m0_gps_longitude = gps_data->longitude;
        m0_gps_altitude = gps_data->altitude;
        m0_gps_fix_quality = gps_data->fix_quality;
        m0_gps_valid = gps_data->gps_valid;
        m0_home_latitude = gps_data->home_lat;
        m0_home_longitude = gps_data->home_lon;
        m0_home_position_set = gps_data->home_set;
        m0_distance_to_home = gps_data->distance_to_home;
        m0_bearing_to_home = gps_data->bearing_to_home;
    }
    
    if (!m0_gps_valid && m0_gps_satellite_count < 4) {
        m0_error_set(0x08); // GPS error
    }
    
    // NEW: Battery and power monitoring
    // Battery voltage monitoring (would read from ADC)
    if (m0_battery_voltage < 11.0f) {
        m0_error_set(0x10); // Low battery
        m0_performance_counters[5]++;
    }
    
    // Power consumption calculation
    m0_power_consumption = (uint32_t)(m0_battery_voltage * m0_battery_current * 1000); // mW
    
    // RAM monitoring
    if (ram_monitor_get_free() < 1024) {
        m0_error_set(0x20); // Low RAM
        m0_performance_counters[6]++;
    }
}

// NEW: M0 functions for enhanced telemetry and monitoring

// Battery monitoring functions
float m0_get_battery_voltage(void) {
    return m0_battery_voltage;
}

float m0_get_battery_current(void) {
    return m0_battery_current;
}

uint32_t m0_get_power_consumption(void) {
    return m0_power_consumption;
}

void m0_update_battery_data(float voltage, float current) {
    m0_battery_voltage = voltage;
    m0_battery_current = current;
}

// GPS and RTH functions
double m0_get_gps_latitude(void) {
    return m0_gps_latitude;
}

double m0_get_gps_longitude(void) {
    return m0_gps_longitude;
}

float m0_get_gps_altitude(void) {
    return m0_gps_altitude;
}

uint8_t m0_get_gps_satellites(void) {
    return m0_gps_satellite_count;
}

bool m0_get_gps_valid(void) {
    return m0_gps_valid;
}

bool m0_get_home_position_set(void) {
    return m0_home_position_set;
}

float m0_get_distance_to_home(void) {
    return m0_distance_to_home;
}

float m0_get_bearing_to_home(void) {
    return m0_bearing_to_home;
}

// RF monitoring functions
uint8_t m0_get_rf_signal_strength(void) {
    return m0_rf_signal_strength;
}

uint16_t m0_get_rf_link_quality(void) {
    return m0_rf_link_quality;
}

uint32_t m0_get_rf_tx_count(void) {
    return m0_rf_tx_count;
}

uint32_t m0_get_rf_rx_count(void) {
    return m0_rf_rx_count;
}

uint32_t m0_get_rf_error_count(void) {
    return m0_rf_error_count;
}

void m0_update_rf_stats(uint32_t tx_count, uint32_t rx_count, uint32_t error_count, uint8_t signal_strength) {
    m0_rf_tx_count = tx_count;
    m0_rf_rx_count = rx_count;
    m0_rf_error_count = error_count;
    m0_rf_signal_strength = signal_strength;
    m0_rf_last_packet_time = 0; // Would get current time
}

// Temperature monitoring functions
float m0_get_temperature(uint8_t sensor_id) {
    if (sensor_id < 8) {
        return m0_temperatures[sensor_id];
    }
    return 0.0f;
}

bool m0_get_thermal_warning(uint8_t sensor_id) {
    if (sensor_id < 8) {
        return m0_thermal_warning_flags[sensor_id];
    }
    return false;
}

bool m0_get_thermal_critical(uint8_t sensor_id) {
    if (sensor_id < 8) {
        return m0_thermal_critical_flags[sensor_id];
    }
    return false;
}

void m0_update_temperature(uint8_t sensor_id, float temperature) {
    if (sensor_id < 8) {
        m0_temperatures[sensor_id] = temperature;
    }
}

// Motor health monitoring functions
bool m0_get_motor_healthy(uint8_t motor_id) {
    if (motor_id < 4) {
        return m0_motor_healthy[motor_id];
    }
    return false;
}

uint8_t m0_get_failed_motor_mask(void) {
    return m0_failed_motor_mask;
}

bool m0_get_thrust_loss_detected(void) {
    return m0_thrust_loss_detected;
}

float m0_get_motor_current(uint8_t motor_id) {
    if (motor_id < 4) {
        return m0_motor_current[motor_id];
    }
    return 0.0f;
}

void m0_update_motor_health(uint8_t motor_id, float current, uint32_t runtime) {
    if (motor_id < 4) {
        m0_motor_current[motor_id] = current;
        m0_motor_runtime[motor_id] = runtime;
    }
}

// CPU load and task monitoring functions
uint16_t m0_get_cpu_load_estimate(void) {
    return m0_cpu_load_estimate;
}

uint16_t m0_get_m4_cpu_utilization(void) {
    return m0_m4_cpu_utilization;
}

uint32_t m0_get_task_execution_time(uint8_t task_id) {
    if (task_id < 8) {
        return m0_task_execution_times[task_id];
    }
    return 0;
}

uint32_t m0_get_task_deadline_misses(uint8_t task_id) {
    if (task_id < 8) {
        return m0_task_deadline_misses[task_id];
    }
    return 0;
}

void m0_update_task_timing(uint8_t task_id, uint32_t execution_time, bool deadline_missed) {
    if (task_id < 8) {
        m0_task_execution_times[task_id] = execution_time;
        if (deadline_missed) {
            m0_task_deadline_misses[task_id]++;
        }
    }
}

void m0_update_cpu_utilization(uint16_t m4_utilization) {
    m0_m4_cpu_utilization = m4_utilization;
}

// Performance counter functions
uint32_t m0_get_performance_counter(uint8_t index) {
    if (index < 16) {
        return m0_performance_counters[index];
    }
    return 0;
}

void m0_reset_performance_counters(void) {
    for (int i = 0; i < 16; i++) {
        m0_performance_counters[i] = 0;
    }
}

void m0_increment_performance_counter(uint8_t index) {
    if (index < 16) {
        m0_performance_counters[index]++;
    }
}

// Telemetry functions
void m0_update_telemetry_stats(uint32_t packets_sent, uint32_t errors) {
    m0_telemetry_packet_count = packets_sent;
    m0_telemetry_error_count = errors;
}

uint32_t m0_get_telemetry_packet_count(void) {
    return m0_telemetry_packet_count;
}

uint32_t m0_get_telemetry_error_count(void) {
    return m0_telemetry_error_count;
}

// System health functions
uint8_t m0_get_system_health_score(void) {
    uint8_t health_score = 100;
    
    // Deduct points for various issues
    if (m0_error_get() != 0) health_score -= 30;                     // Active errors
    if (m0_battery_voltage < 11.5f) health_score -= 20;             // Low battery
    if (m0_rf_link_quality < 50) health_score -= 15;                // Poor RF
    if (m0_gps_satellite_count < 4) health_score -= 10;             // Poor GPS
    if (m0_cpu_load_estimate > 80) health_score -= 15;              // High CPU
    if (m0_failed_motor_mask != 0) health_score -= 25;              // Motor failures
    
    for (int i = 0; i < 8; i++) {
        if (m0_thermal_warning_flags[i]) health_score -= 5;         // Thermal warnings
        if (m0_thermal_critical_flags[i]) health_score -= 20;       // Thermal critical
    }
    
    return (health_score < 0) ? 0 : health_score;
}

bool m0_is_system_safe_for_flight(void) {
    // Comprehensive flight safety check
    return (m0_get_system_health_score() > 70 &&
            m0_battery_voltage > 11.0f &&
            m0_failed_motor_mask == 0 &&
            !m0_thrust_loss_detected &&
            m0_rf_link_quality > 30);
}