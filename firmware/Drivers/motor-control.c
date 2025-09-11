/*
 * motor_control.c - Motor/ESC control for quadcopter (matches existing codebase structure)
 * 
 * This file provides PWM-based motor control for the LPC4330 dual-core drone system
 * Following the existing driver pattern used in the codebase
 */
#include "motor_control.h"
#include "logging.h"
#include "chip.h"  // LPC4330 chip definitions
#include "spi_bus.h"  // For error handling patterns
#include <string.h>

// Motor configuration - follows existing pin config pattern
#define MOTOR_PWM_FREQ_HZ 400      // 400Hz for modern ESCs (DShot compatible)
#define MOTOR_PWM_PERIOD_US 2500   // 2.5ms period

// Motor pin assignments - using existing SCU pin mux pattern
#define MOTOR1_PWM_PIN_PORT 4
#define MOTOR1_PWM_PIN_NUM  1      // P4_1 -> PWM1.1
#define MOTOR2_PWM_PIN_PORT 4  
#define MOTOR2_PWM_PIN_NUM  2      // P4_2 -> PWM1.2
#define MOTOR3_PWM_PIN_PORT 4
#define MOTOR3_PWM_PIN_NUM  3      // P4_3 -> PWM1.3
#define MOTOR4_PWM_PIN_PORT 4
#define MOTOR4_PWM_PIN_NUM  4      // P4_4 -> PWM1.4

// Motor control state - follows existing driver pattern
static bool motor_control_initialized = false;
static bool motors_armed = false;
static motor_outputs_t current_outputs = {0};
static uint32_t motor_error_count = 0;

// Internal functions
static int motor_control_setup_pwm(void);
static void motor_control_update_pwm(const motor_outputs_t *outputs);
static bool motor_control_validate_outputs(const motor_outputs_t *outputs);

int motor_control_init(void) {
    log_info("Initializing motor control system...");
    
    if (motor_control_initialized) {
        log_warning("Motor control already initialized");
        return 0;
    }
    
    // Initialize PWM peripheral - follows existing init pattern
    if (motor_control_setup_pwm() != 0) {
        log_error("Failed to setup PWM for motor control");
        return -1;
    }
    
    // Start with all motors disarmed
    motor_control_disarm();
    
    motor_control_initialized = true;
    motor_error_count = 0;
    
    log_info("Motor control initialized successfully");
    return 0;
}

static int motor_control_setup_pwm(void) {
    // Configure PWM pins - follows existing SCU pin config pattern from drivers
    Chip_SCU_PinMuxSet(MOTOR1_PWM_PIN_PORT, MOTOR1_PWM_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_FUNC1));
    Chip_SCU_PinMuxSet(MOTOR2_PWM_PIN_PORT, MOTOR2_PWM_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_FUNC1));  
    Chip_SCU_PinMuxSet(MOTOR3_PWM_PIN_PORT, MOTOR3_PWM_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_FUNC1));
    Chip_SCU_PinMuxSet(MOTOR4_PWM_PIN_PORT, MOTOR4_PWM_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_FUNC1));
    
    // Initialize PWM peripheral
    Chip_PWM_Init(LPC_PWM1);
    
    // Set PWM frequency
    Chip_PWM_SetFreq(LPC_PWM1, MOTOR_PWM_FREQ_HZ);
    
    // Configure PWM channels
    Chip_PWM_SetupChannel(LPC_PWM1, 1, PWM_SINGLE_EDGE);
    Chip_PWM_SetupChannel(LPC_PWM1, 2, PWM_SINGLE_EDGE);
    Chip_PWM_SetupChannel(LPC_PWM1, 3, PWM_SINGLE_EDGE);
    Chip_PWM_SetupChannel(LPC_PWM1, 4, PWM_SINGLE_EDGE);
    
    // Enable PWM channels
    Chip_PWM_EnableChannels(LPC_PWM1, PWM_OUT1 | PWM_OUT2 | PWM_OUT3 | PWM_OUT4);
    
    // Start PWM
    Chip_PWM_Start(LPC_PWM1);
    
    return 0;
}

void motor_control_set_outputs(const motor_outputs_t *outputs) {
    if (!motor_control_initialized || outputs == NULL) {
        motor_error_count++;
        return;
    }
    
    // Validate outputs - follows existing error handling pattern
    if (!motor_control_validate_outputs(outputs)) {
        motor_error_count++;
        log_warning("Invalid motor outputs rejected");
        return;
    }
    
    // Apply safety limits and disarm check
    motor_outputs_t safe_outputs = *outputs;
    
    if (!motors_armed) {
        // Force all outputs to zero when disarmed
        safe_outputs.motor1 = MOTOR_DISARMED_PWM;
        safe_outputs.motor2 = MOTOR_DISARMED_PWM;
        safe_outputs.motor3 = MOTOR_DISARMED_PWM;
        safe_outputs.motor4 = MOTOR_DISARMED_PWM;
    } else {
        // Apply safety limits when armed
        if (safe_outputs.motor1 > MOTOR_MAX_PWM) safe_outputs.motor1 = MOTOR_MAX_PWM;
        if (safe_outputs.motor2 > MOTOR_MAX_PWM) safe_outputs.motor2 = MOTOR_MAX_PWM;
        if (safe_outputs.motor3 > MOTOR_MAX_PWM) safe_outputs.motor3 = MOTOR_MAX_PWM;
        if (safe_outputs.motor4 > MOTOR_MAX_PWM) safe_outputs.motor4 = MOTOR_MAX_PWM;
        
        // Ensure minimum speed when armed
        if (safe_outputs.motor1 > 0 && safe_outputs.motor1 < MOTOR_IDLE_PWM) 
            safe_outputs.motor1 = MOTOR_IDLE_PWM;
        if (safe_outputs.motor2 > 0 && safe_outputs.motor2 < MOTOR_IDLE_PWM) 
            safe_outputs.motor2 = MOTOR_IDLE_PWM;
        if (safe_outputs.motor3 > 0 && safe_outputs.motor3 < MOTOR_IDLE_PWM) 
            safe_outputs.motor3 = MOTOR_IDLE_PWM;
        if (safe_outputs.motor4 > 0 && safe_outputs.motor4 < MOTOR_IDLE_PWM) 
            safe_outputs.motor4 = MOTOR_IDLE_PWM;
    }
    
    // Update PWM outputs
    motor_control_update_pwm(&safe_outputs);
    
    // Store current outputs
    current_outputs = safe_outputs;
}

static void motor_control_update_pwm(const motor_outputs_t *outputs) {
    // Convert PWM values (in microseconds) to timer match values
    // Follows existing timer calculation pattern from other drivers
    
    uint32_t pwm_period = SystemCoreClock / MOTOR_PWM_FREQ_HZ;
    
    uint32_t motor1_match = (outputs->motor1 * pwm_period) / (MOTOR_PWM_PERIOD_US);
    uint32_t motor2_match = (outputs->motor2 * pwm_period) / (MOTOR_PWM_PERIOD_US);
    uint32_t motor3_match = (outputs->motor3 * pwm_period) / (MOTOR_PWM_PERIOD_US);
    uint32_t motor4_match = (outputs->motor4 * pwm_period) / (MOTOR_PWM_PERIOD_US);
    
    // Set PWM duty cycles
    Chip_PWM_SetDutyCycle(LPC_PWM1, 1, motor1_match);
    Chip_PWM_SetDutyCycle(LPC_PWM1, 2, motor2_match);
    Chip_PWM_SetDutyCycle(LPC_PWM1, 3, motor3_match);
    Chip_PWM_SetDutyCycle(LPC_PWM1, 4, motor4_match);
    
    // Latch new values - follows existing PWM pattern
    Chip_PWM_LatchEnable(LPC_PWM1, PWM_OUT1 | PWM_OUT2 | PWM_OUT3 | PWM_OUT4);
}

static bool motor_control_validate_outputs(const motor_outputs_t *outputs) {
    // Validate all motor values are within acceptable range
    if (outputs->motor1 > MOTOR_MAX_PWM || outputs->motor2 > MOTOR_MAX_PWM ||
        outputs->motor3 > MOTOR_MAX_PWM || outputs->motor4 > MOTOR_MAX_PWM) {
        return false;
    }
    
    // Check for reasonable values (not negative, etc.)
    if (outputs->motor1 < 0 || outputs->motor2 < 0 ||
        outputs->motor3 < 0 || outputs->motor4 < 0) {
        return false;
    }
    
    return true;
}

void motor_control_mix_inputs(const motor_mix_t *inputs, motor_outputs_t *outputs) {
    if (!inputs || !outputs) {
        return;
    }
    
    // Quadcopter motor mixing matrix (X configuration)
    // Motor layout:     Front
    //                 2     1
    //                  \ X /
    //                  / X \  
    //                 3     4
    //                  Back
    
    // Motor mixing equations for X quadcopter configuration
    float motor1_mix = inputs->throttle + inputs->pitch - inputs->roll - inputs->yaw; // Front Right
    float motor2_mix = inputs->throttle + inputs->pitch + inputs->roll + inputs->yaw; // Front Left
    float motor3_mix = inputs->throttle - inputs->pitch + inputs->roll - inputs->yaw; // Back Left  
    float motor4_mix = inputs->throttle - inputs->pitch - inputs->roll + inputs->yaw; // Back Right
    
    // Normalize to prevent saturation - follows existing algorithm pattern
    float max_mix = motor1_mix;
    if (motor2_mix > max_mix) max_mix = motor2_mix;
    if (motor3_mix > max_mix) max_mix = motor3_mix;
    if (motor4_mix > max_mix) max_mix = motor4_mix;
    
    if (max_mix > 1.0f) {
        float scale = 1.0f / max_mix;
        motor1_mix *= scale;
        motor2_mix *= scale; 
        motor3_mix *= scale;
        motor4_mix *= scale;
    }
    
    // Ensure minimum values
    if (motor1_mix < 0.0f) motor1_mix = 0.0f;
    if (motor2_mix < 0.0f) motor2_mix = 0.0f;
    if (motor3_mix < 0.0f) motor3_mix = 0.0f;
    if (motor4_mix < 0.0f) motor4_mix = 0.0f;
    
    // Convert to PWM microseconds (1000-2000μs range)
    outputs->motor1 = MOTOR_MIN_PWM + (uint16_t)(motor1_mix * (MOTOR_MAX_PWM - MOTOR_MIN_PWM));
    outputs->motor2 = MOTOR_MIN_PWM + (uint16_t)(motor2_mix * (MOTOR_MAX_PWM - MOTOR_MIN_PWM));
    outputs->motor3 = MOTOR_MIN_PWM + (uint16_t)(motor3_mix * (MOTOR_MAX_PWM - MOTOR_MIN_PWM));
    outputs->motor4 = MOTOR_MIN_PWM + (uint16_t)(motor4_mix * (MOTOR_MAX_PWM - MOTOR_MIN_PWM));
}

void motor_control_disarm(void) {
    motors_armed = false;
    
    motor_outputs_t disarmed_outputs = {
        .motor1 = MOTOR_DISARMED_PWM,
        .motor2 = MOTOR_DISARMED_PWM,
        .motor3 = MOTOR_DISARMED_PWM,
        .motor4 = MOTOR_DISARMED_PWM
    };
    
    motor_control_set_outputs(&disarmed_outputs);
    log_info("Motors disarmed");
}

int motor_control_arm(void) {
    if (!motor_control_initialized) {
        log_error("Cannot arm - motor control not initialized");
        return -1;
    }
    
    // Safety check - ensure throttle is at minimum
    if (current_outputs.motor1 > MOTOR_IDLE_PWM || current_outputs.motor2 > MOTOR_IDLE_PWM ||
        current_outputs.motor3 > MOTOR_IDLE_PWM || current_outputs.motor4 > MOTOR_IDLE_PWM) {
        log_error("Cannot arm - motors not at idle");
        return -1;
    }
    
    motors_armed = true;
    log_info("Motors armed");
    return 0;
}

void motor_control_test_sequence(void) {
    if (!motor_control_initialized) {
        log_error("Motor control not initialized");
        return;
    }
    
    log_info("Starting motor test sequence - ENSURE PROPELLERS ARE REMOVED");
    
    // Test each motor individually at low speed
    motor_outputs_t test_outputs = {0};
    const uint16_t test_speed = MOTOR_IDLE_PWM + 50; // Very low speed
    const uint32_t test_duration_ms = 2000; // 2 seconds per motor
    
    // Temporarily arm for testing
    bool was_armed = motors_armed;
    motors_armed = true;
    
    // Test Motor 1
    log_info("Testing Motor 1");
    test_outputs.motor1 = test_speed;
    motor_control_set_outputs(&test_outputs);
    vTaskDelay(pdMS_TO_TICKS(test_duration_ms));
    test_outputs.motor1 = MOTOR_IDLE_PWM;
    motor_control_set_outputs(&test_outputs);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test Motor 2
    log_info("Testing Motor 2");
    test_outputs.motor2 = test_speed;
    motor_control_set_outputs(&test_outputs);
    vTaskDelay(pdMS_TO_TICKS(test_duration_ms));
    test_outputs.motor2 = MOTOR_IDLE_PWM;
    motor_control_set_outputs(&test_outputs);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test Motor 3
    log_info("Testing Motor 3");
    test_outputs.motor3 = test_speed;
    motor_control_set_outputs(&test_outputs);
    vTaskDelay(pdMS_TO_TICKS(test_duration_ms));
    test_outputs.motor3 = MOTOR_IDLE_PWM;
    motor_control_set_outputs(&test_outputs);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test Motor 4
    log_info("Testing Motor 4");
    test_outputs.motor4 = test_speed;
    motor_control_set_outputs(&test_outputs);
    vTaskDelay(pdMS_TO_TICKS(test_duration_ms));
    test_outputs.motor4 = MOTOR_IDLE_PWM;
    motor_control_set_outputs(&test_outputs);
    
    // Restore armed state
    motors_armed = was_armed;
    
    // Return to disarmed outputs
    motor_control_disarm();
    
    log_info("Motor test sequence completed");
}

int motor_control_get_status(void) {
    if (!motor_control_initialized) {
        return -1;
    }
    
    return motors_armed ? 1 : 0;
}

uint32_t motor_control_get_error_count(void) {
    return motor_error_count;
}

void motor_control_clear_errors(void) {
    motor_error_count = 0;
}

const motor_outputs_t* motor_control_get_current_outputs(void) {
    return &current_outputs;
}