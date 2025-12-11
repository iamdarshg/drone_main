// Enhanced M0 watchdog with telemetry and performance monitoring offloaded functions
// This extends the existing watchdog_m0.c with additional monitoring capabilities

#include "watchdog_m0.h"
#include "selftest_m0.h"
#include "bus_monitor.h"
#include "imu_monitor.h"
#include "rf_monitor.h"
#include "ram_monitor.h"
#include "core_monitor.h"
#include "gps_uart.h"
#include <stdint.h>

// Enhanced M0 state - extends existing functionality
volatile uint32_t m0_watchdog_counter = 0;
volatile uint32_t m0_status_flags = 0;
volatile uint32_t m0_pid_delay_flag = 0;  
volatile uint32_t m0_task_priority_flags = 0;

// New telemetry and performance monitoring variables - offloaded to M0
volatile uint32_t m0_telemetry_packet_count = 0;
volatile uint32_t m0_telemetry_error_count = 0;
volatile uint16_t m0_cpu_load_estimate = 10;
volatile uint32_t m0_performance_counters[8] = {0}; // Various performance metrics

// Battery monitoring - offloaded to M0 core
volatile float m0_battery_voltage = 12.6f;
volatile uint16_t m0_battery_adc_raw = 0;

// GPS monitoring - offloaded to M0 core  
volatile uint8_t m0_gps_satellite_count = 0;
volatile float m0_gps_latitude = 0.0f;
volatile float m0_gps_longitude = 0.0f;
volatile uint8_t m0_gps_fix_quality = 0;

// RF monitoring - offloaded to M0 core
volatile uint8_t m0_rf_signal_strength = 0;
volatile uint32_t m0_rf_tx_count = 0;
volatile uint32_t m0_rf_rx_count = 0;
volatile uint32_t m0_rf_error_count = 0;

// Task timing monitoring - offloaded to M0
volatile uint32_t m0_task_timing[5] = {0}; // Flight, IMU, Command, Telemetry, Safety tasks

// Existing M0 functions with enhancements
void m0_watchdog_kick(void) {
    m0_watchdog_counter = 0;
}

void m0_watchdog_task(void) {
    // Enhanced watchdog task with performance monitoring
    m0_watchdog_counter++;
    
    // Update CPU load estimate based on activity
    m0_cpu_load_estimate = 10 + (m0_watchdog_counter % 20); // Simple estimate
    
    if (m0_watchdog_counter > 1000000) {
        m0_status_flags |= 0x01; // Set timeout flag
        // Optionally trigger system reset or alert M4
    }
    
    // Increment performance counter
    m0_performance_counters[0]++; // Watchdog iterations
}

void m0_monitor_pid_loop(uint32_t last_pid_tick, uint32_t now, uint32_t max_delay) {
    if ((now - last_pid_tick) > max_delay) {
        m0_pid_delay_flag = 1;
        m0_performance_counters[1]++; // PID delay events
    } else {
        m0_pid_delay_flag = 0;
    }
    
    // Store timing for telemetry
    m0_task_timing[0] = now - last_pid_tick; // Flight control task timing
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

// Enhanced hardware checking with telemetry collection
void m0_check_all_hardware(void) {
    // Bus contention monitoring
    if (bus_monitor_get_spi_bytes() > 1000000 || bus_monitor_get_i2c_bytes() > 1000000) {
        m0_error_set(0x01); // Bus error
        bus_monitor_reset();
        m0_performance_counters[2]++; // Bus error count
    }

    // IMU lockup detection
    imu_monitor_check_all(); 
    if (imu_monitor_get_errors()) {
        m0_error_set(0x02);
        m0_performance_counters[3]++; // IMU error count
    }

    // IMU FIFO and ODR validation
    #ifdef IMU_KX122_PRESENT
    if (!imu_kx122_fifo_rate_supported() || !imu_kx122_odr_supported()) {
        m0_error_set(0x20); // KX122 FIFO/ODR error
    }
    #endif
    
    #ifdef IMU_LSM6DS3_PRESENT
    if (!imu_lsm6ds3_fifo_rate_supported() || !imu_lsm6ds3_odr_supported()) {
        m0_error_set(0x40); // LSM6DS3 FIFO/ODR error
    }
    #endif
    
    #ifdef IMU_LSM303C_PRESENT
    if (!imu_lsm303c_fifo_rate_supported() || !imu_lsm303c_odr_supported()) {
        m0_error_set(0x80); // LSM303C FIFO/ODR error
    }
    #endif

    // RF monitoring - collect statistics for telemetry
    m0_rf_tx_count = rf_monitor_get_tx_count();
    m0_rf_rx_count = rf_monitor_get_rx_count(); 
    m0_rf_error_count = rf_monitor_get_errors();
    
    if (m0_rf_error_count > 10) {
        m0_error_set(0x04); // RF desync error
        m0_performance_counters[4]++; // RF error events
    }
    
    // Estimate RF signal strength (would be implemented in rf_monitor)
    m0_rf_signal_strength = 85 - (m0_rf_error_count * 2); // Simple estimate
    if (m0_rf_signal_strength > 100) m0_rf_signal_strength = 100;

    // GPS monitoring - offloaded to M0
    char nmea[128];
    if (gps_uart_read_line(nmea, sizeof(nmea)) < 0) {
        m0_error_set(0x08); // GPS UART framing error
    } else {
        // Parse basic GPS info (simplified)
        // In real implementation, would parse NMEA sentences
        m0_gps_satellite_count = 8; // Placeholder
        m0_gps_fix_quality = 1;     // Placeholder
    }

    // RAM monitoring  
    if (ram_monitor_get_free() < 1024) {
        m0_error_set(0x10); // RAM overflow
        m0_performance_counters[5]++; // RAM critical events
    }

    // Battery monitoring (offloaded ADC reading to M0)
    // This would read ADC and convert to voltage
    m0_battery_adc_raw = 2048; // Placeholder ADC reading
    m0_battery_voltage = (m0_battery_adc_raw * 3.3f * 4.0f) / 4096.0f; // Simple conversion
    
    if (m0_battery_voltage < 11.0f) {
        m0_error_set(0x100); // Low battery
        m0_performance_counters[6]++; // Low battery events
    }
}

// New M0 functions for telemetry support
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

uint16_t m0_get_cpu_load_estimate(void) {
    return m0_cpu_load_estimate;
}

// Battery monitoring functions - offloaded to M0
float m0_get_battery_voltage(void) {
    return m0_battery_voltage;
}

uint16_t m0_get_battery_adc_raw(void) {
    return m0_battery_adc_raw;
}

// GPS functions - offloaded to M0
uint8_t m0_get_gps_satellite_count(void) {
    return m0_gps_satellite_count;
}

float m0_get_gps_latitude(void) {
    return m0_gps_latitude;
}

float m0_get_gps_longitude(void) {
    return m0_gps_longitude;
}

uint8_t m0_get_gps_fix_quality(void) {
    return m0_gps_fix_quality;
}

// RF monitoring functions - offloaded to M0
uint8_t m0_get_rf_signal_strength(void) {
    return m0_rf_signal_strength;
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

// Performance counter access
uint32_t m0_get_performance_counter(uint8_t index) {
    if (index < 8) {
        return m0_performance_counters[index];
    }
    return 0;
}

void m0_reset_performance_counters(void) {
    for (int i = 0; i < 8; i++) {
        m0_performance_counters[i] = 0;
    }
}

// Task timing monitoring - offloaded to M0
void m0_update_task_timing(uint8_t task_id, uint32_t execution_time) {
    if (task_id < 5) {
        m0_task_timing[task_id] = execution_time;
    }
}

uint32_t m0_get_task_timing(uint8_t task_id) {
    if (task_id < 5) {
        return m0_task_timing[task_id];
    }
    return 0;
}

// System health summary - computed on M0 to reduce M4 load
uint8_t m0_get_system_health_score(void) {
    uint8_t health_score = 100;
    
    // Deduct points for various issues
    if (m0_error_get() != 0) health_score -= 20;           // Active errors
    if (m0_battery_voltage < 11.5f) health_score -= 15;    // Low battery
    if (m0_rf_error_count > 5) health_score -= 10;         // RF issues
    if (m0_gps_satellite_count < 4) health_score -= 5;     // Poor GPS
    if (m0_cpu_load_estimate > 80) health_score -= 10;     // High CPU load
    
    return health_score;
}