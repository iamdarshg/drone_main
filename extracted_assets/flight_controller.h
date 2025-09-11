/*
 * Enhanced flight controller with all safety features
 * 
 * Provides stub implementations for the enhanced flight control functions
 * that are called from the main control loop
 */
#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

// Forward declarations matching the enhanced init.c structures
typedef struct {
    float roll, pitch, yaw;
    float roll_rate, pitch_rate, yaw_rate;
    bool valid;
} attitude_t;

typedef struct {
    float throttle;
    float roll_cmd, pitch_cmd, yaw_cmd;
    uint8_t flight_mode;
    bool arm_request;
    uint32_t timestamp;
} command_t;

typedef struct {
    uint16_t motor1, motor2, motor3, motor4;
} motor_outputs_t;

// Flight controller functions
int flight_controller_stabilize(const attitude_t *attitude, const command_t *command, motor_outputs_t *outputs);
int flight_controller_altitude_hold(const attitude_t *attitude, const command_t *command, motor_outputs_t *outputs);
int flight_controller_autonomous(const attitude_t *attitude, const command_t *command, motor_outputs_t *outputs);

// Safety monitoring functions
typedef enum {
    SAFETY_OK = 0,
    SAFETY_IMU_FAILURE,
    SAFETY_ATTITUDE_INVALID,
    SAFETY_MOTOR_FAILURE,
    SAFETY_RF_TIMEOUT,
    SAFETY_BATTERY_LOW,
    SAFETY_THERMAL_WARNING,
    SAFETY_GPS_LOST,
    SAFETY_CRITICAL_ERROR
} safety_status_t;

int safety_monitor_init(void);
safety_status_t safety_monitor_check(void);

// Command processor functions
typedef struct {
    float throttle;
    float roll_cmd, pitch_cmd, yaw_cmd;
    uint8_t flight_mode;
    bool arm_request;
    uint32_t timestamp;
} command_t;

int command_processor_init(void);
int command_processor_parse(const uint8_t *data, uint16_t length, command_t *output);
bool command_processor_is_timeout(void);

// RF communication functions
int rf_receive_hamming(uint8_t *buffer, size_t buffer_size);
int rf_send_hamming(const uint8_t *data, size_t length);

// Telemetry functions with enhanced data
typedef struct {
    uint16_t header;
    uint32_t timestamp;
    
    // Attitude data
    float roll, pitch, yaw;
    float roll_rate, pitch_rate, yaw_rate;
    
    // Command inputs
    float throttle_cmd, roll_cmd, pitch_cmd, yaw_cmd;
    
    // Motor outputs
    uint16_t motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
    
    // Enhanced telemetry data
    double gps_latitude, gps_longitude;
    uint8_t gps_satellites;
    float distance_to_home;
    float m4_core_temp;
    bool thermal_warning;
    uint8_t motor_health_mask;
    bool thrust_loss_detected;
    
    // System status
    uint8_t flight_mode, system_status;
    uint16_t cpu_load_m4, cpu_load_m0;
    uint16_t m0_status;
    uint32_t m0_errors;
    float battery_voltage;
    
    uint16_t crc16;
} telemetry_data_t;

int telemetry_init(void);
int telemetry_transmit(const telemetry_data_t *data);

#endif // FLIGHT_CONTROLLER_H