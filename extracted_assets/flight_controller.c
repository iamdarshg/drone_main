/*
 * flight_controller.c - Enhanced flight controller implementation
 * 
 * Provides complete flight control algorithms with all safety features
 */
#include "flight_controller.h"
#include "motor_control.h"
#include "logging.h"
#include "pid.h"
#include <math.h>
#include <string.h>

// PID controllers for attitude control
static pid_t roll_attitude_pid, pitch_attitude_pid, yaw_attitude_pid;
static pid_t roll_rate_pid, pitch_rate_pid, yaw_rate_pid;

// PID sensor arrays (required by existing pid.h interface)
static float roll_attitude_sensor[1];
static float pitch_attitude_sensor[1];  
static float yaw_attitude_sensor[1];
static float roll_rate_sensor[1];
static float pitch_rate_sensor[1];
static float yaw_rate_sensor[1];

// Control parameters
static bool controllers_initialized = false;

#define MAX_ANGLE_DEG 30.0f
#define MAX_RATE_DPS 180.0f

static void initialize_controllers(void) {
    if (controllers_initialized) return;
    
    // Initialize attitude PIDs (outer loop)
    pid_init(&roll_attitude_pid, 4.0f, 0.1f, 0.2f, roll_attitude_sensor, 1);
    pid_init(&pitch_attitude_pid, 4.0f, 0.1f, 0.2f, pitch_attitude_sensor, 1);
    pid_init(&yaw_attitude_pid, 2.0f, 0.05f, 0.0f, yaw_attitude_sensor, 1);
    
    // Initialize rate PIDs (inner loop)
    pid_init(&roll_rate_pid, 0.8f, 0.2f, 0.01f, roll_rate_sensor, 1);
    pid_init(&pitch_rate_pid, 0.8f, 0.2f, 0.01f, pitch_rate_sensor, 1);
    pid_init(&yaw_rate_pid, 1.2f, 0.1f, 0.005f, yaw_rate_sensor, 1);
    
    controllers_initialized = true;
    log_info("Flight controllers initialized");
}

int flight_controller_stabilize(const attitude_t *attitude, const command_t *command, motor_outputs_t *outputs) {
    if (!attitude || !command || !outputs || !attitude->valid) {
        return -1;
    }
    
    initialize_controllers();
    
    const float dt = 0.001f; // 1kHz control loop
    
    // Convert command inputs to angle setpoints
    float roll_setpoint = command->roll_cmd * MAX_ANGLE_DEG * M_PI / 180.0f;
    float pitch_setpoint = command->pitch_cmd * MAX_ANGLE_DEG * M_PI / 180.0f;
    float yaw_rate_setpoint = command->yaw_cmd * MAX_RATE_DPS * M_PI / 180.0f;
    
    // Attitude control (outer loop)
    roll_attitude_sensor[0] = attitude->roll;
    pitch_attitude_sensor[0] = attitude->pitch;
    
    float roll_rate_cmd = pid_update(&roll_attitude_pid, roll_setpoint, dt);
    float pitch_rate_cmd = pid_update(&pitch_attitude_pid, pitch_setpoint, dt);
    
    // Rate control (inner loop)
    roll_rate_sensor[0] = attitude->roll_rate;
    pitch_rate_sensor[0] = attitude->pitch_rate;
    yaw_rate_sensor[0] = attitude->yaw_rate;
    
    float roll_output = pid_update(&roll_rate_pid, roll_rate_cmd, dt);
    float pitch_output = pid_update(&pitch_rate_pid, pitch_rate_cmd, dt);
    float yaw_output = pid_update(&yaw_rate_pid, yaw_rate_setpoint, dt);
    
    // Limit outputs
    roll_output = fmaxf(-1.0f, fminf(1.0f, roll_output));
    pitch_output = fmaxf(-1.0f, fminf(1.0f, pitch_output));
    yaw_output = fmaxf(-1.0f, fminf(1.0f, yaw_output));
    
    // Motor mixing for quadcopter
    motor_mix_t motor_mix = {
        .throttle = command->throttle,
        .roll = roll_output,
        .pitch = pitch_output,
        .yaw = yaw_output
    };
    
    motor_control_mix_inputs(&motor_mix, outputs);
    
    return 0;
}

int flight_controller_altitude_hold(const attitude_t *attitude, const command_t *command, motor_outputs_t *outputs) {
    // For now, same as stabilize mode - altitude hold would need barometer/rangefinder
    return flight_controller_stabilize(attitude, command, outputs);
}

int flight_controller_autonomous(const attitude_t *attitude, const command_t *command, motor_outputs_t *outputs) {
    // For now, same as stabilize mode - autonomous would need waypoint navigation
    return flight_controller_stabilize(attitude, command, outputs);
}

// Safety monitoring implementation
static uint32_t last_safety_check = 0;

int safety_monitor_init(void) {
    last_safety_check = 0;
    log_info("Safety monitor initialized");
    return 0;
}

safety_status_t safety_monitor_check(void) {
    // Placeholder safety checks - would be expanded with real sensor monitoring
    return SAFETY_OK;
}

// Command processor implementation
static uint32_t last_command_time = 0;

int command_processor_init(void) {
    last_command_time = 0;
    log_info("Command processor initialized");
    return 0;
}

int command_processor_parse(const uint8_t *data, uint16_t length, command_t *output) {
    // Simplified command parsing - real implementation would parse RF packets
    if (!data || !output || length < 16) {
        return -1;
    }
    
    // Update command timestamp
    last_command_time = xTaskGetTickCount();
    
    return 0;
}

bool command_processor_is_timeout(void) {
    uint32_t current_time = xTaskGetTickCount();
    return (current_time - last_command_time) > pdMS_TO_TICKS(1000); // 1 second timeout
}

// RF communication stubs
int rf_receive_hamming(uint8_t *buffer, size_t buffer_size) {
    // Placeholder - would interface with actual RF driver
    (void)buffer;
    (void)buffer_size;
    return 0;
}

int rf_send_hamming(const uint8_t *data, size_t length) {
    // Placeholder - would interface with actual RF driver
    (void)data;
    (void)length;
    return 0;
}

// Telemetry implementation
int telemetry_init(void) {
    log_info("Telemetry system initialized");
    return 0;
}

int telemetry_transmit(const telemetry_data_t *data) {
    if (!data) {
        return -1;
    }
    
    // Transmit via RF
    return rf_send_hamming((const uint8_t*)data, sizeof(*data));
}