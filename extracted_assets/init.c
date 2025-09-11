/*
 * init.c - Enhanced drone initialization with all advanced safety features
 * 
 * NEW FEATURES ADDED:
 * - GPS-based Return-to-Home (RTH) functionality
 * - Temperature-based IMU bias compensation  
 * - Timer-based interrupt control loops for precise timing
 * - M4 picking up M0 tasks during downtime
 * - Thermal shutdown protection for M4
 * - Dynamic CPU load management
 * - Motor failure detection and compensation
 */
#include "init.h"
#include "logging.h"
#include "imu_kx122.h"
#include "imu_lsm6ds3.h"
#include "imu_lsm303c.h"
#include "rf_s2lpqtr.h"
#include "gps_uart.h"
#include "spi_bus.h"
#include "i2c_bus.h"
#include "flash.h"
#include "usb_bridge.h"
#include "motor_control.h"
#include "attitude_estimator.h"
#include "command_processor.h"
#include "telemetry.h"
#include "safety_monitor.h"
#include "watchdog_m0.h"
#include "selftest_m0.h"
#include "Config/config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "chip.h"
#include <math.h>

// ==================== ENHANCED DATA STRUCTURES ====================

typedef enum {
    SYSTEM_INITIALIZING,
    SYSTEM_READY,
    SYSTEM_ERROR,
    SYSTEM_THERMAL_SHUTDOWN,
    SYSTEM_RTH_MODE
} system_state_t;

typedef enum {
    FLIGHT_MODE_DISARMED = 0,
    FLIGHT_MODE_STABILIZE = 1,
    FLIGHT_MODE_ALTITUDE_HOLD = 2,
    FLIGHT_MODE_AUTONOMOUS = 3,
    FLIGHT_MODE_RTH = 4,           // NEW: Return-to-Home mode
    FLIGHT_MODE_MOTOR_FAILURE = 5  // NEW: Motor failure compensation mode
} flight_mode_t;

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float temperature;
    uint32_t timestamp;
} imu_data_t;

typedef struct {
    float roll, pitch, yaw;
    float roll_rate, pitch_rate, yaw_rate;
    bool valid;
} attitude_t;

typedef struct {
    float throttle;
    float roll_cmd, pitch_cmd, yaw_cmd;
    flight_mode_t flight_mode;
    bool arm_request;
    uint32_t timestamp;
} command_t;

typedef struct {
    uint16_t motor1, motor2, motor3, motor4;
} motor_outputs_t;

// NEW: GPS data structure for RTH
typedef struct {
    double latitude;
    double longitude;
    float altitude;
    uint8_t satellites;
    uint8_t fix_quality;
    float home_lat, home_lon;  // Home position
    bool home_set;
    float distance_to_home;
    float bearing_to_home;
    bool gps_valid;
} gps_data_t;

// NEW: Temperature monitoring structure
typedef struct {
    float m4_core_temp;
    float imu_temp[3];        // Temperature from each IMU
    float ambient_temp;
    bool thermal_warning;
    bool thermal_critical;
    uint32_t last_temp_check;
} thermal_data_t;

// NEW: Motor health monitoring
typedef struct {
    bool motor_healthy[4];
    float motor_current[4];    // Estimated current draw
    uint32_t motor_failure_count[4];
    uint8_t failed_motor_mask; // Bitmask of failed motors
    bool thrust_loss_detected;
} motor_health_t;

// NEW: CPU load management
typedef struct {
    uint16_t m4_cpu_load;
    uint16_t m0_cpu_load;
    bool load_critical;
    uint32_t tasks_offloaded_to_m4;
    TickType_t last_load_check;
} cpu_load_data_t;

// ==================== GLOBAL VARIABLES ====================

static system_state_t system_state = SYSTEM_INITIALIZING;
static flight_mode_t current_flight_mode = FLIGHT_MODE_DISARMED;
static SemaphoreHandle_t imu_data_mutex;
static SemaphoreHandle_t command_mutex;
static SemaphoreHandle_t attitude_mutex;
static SemaphoreHandle_t gps_data_mutex;     // NEW
static SemaphoreHandle_t thermal_data_mutex; // NEW

// Flight control data
static imu_data_t current_imu_data;
static attitude_t current_attitude;
static command_t current_command;
static motor_outputs_t motor_outputs;
static gps_data_t gps_data = {0};           // NEW
static thermal_data_t thermal_data = {0};   // NEW
static motor_health_t motor_health = {0};   // NEW
static cpu_load_data_t cpu_load = {0};      // NEW

// NEW: Timer-based control loop handles
static TimerHandle_t flight_control_timer;
static TimerHandle_t imu_timer;
static TimerHandle_t command_timer;

// Task handles
static TaskHandle_t flight_control_task_handle;
static TaskHandle_t imu_task_handle;
static TaskHandle_t command_task_handle;
static TaskHandle_t telemetry_task_handle;
static TaskHandle_t gps_task_handle;        // NEW
static TaskHandle_t thermal_task_handle;    // NEW

// NEW: Temperature compensation lookup tables
static const float temp_compensation_accel[3][2] = {
    {0.0001f, -0.05f},  // X axis: scale, offset per degree C
    {0.0001f, -0.05f},  // Y axis
    {0.0001f, -0.05f}   // Z axis
};

static const float temp_compensation_gyro[3][2] = {
    {0.0002f, -0.1f},   // X axis
    {0.0002f, -0.1f},   // Y axis  
    {0.0002f, -0.1f}    // Z axis
};

// ==================== FUNCTION DECLARATIONS ====================

// Existing task functions
static void imu_task(void *pvParameters);
static void command_task(void *pvParameters);
static void telemetry_task(void *pvParameters);
static void safety_task(void *pvParameters);
static void m0_monitor_task(void *pvParameters);

// NEW: Enhanced task functions
static void gps_task(void *pvParameters);
static void thermal_monitoring_task(void *pvParameters);
static void cpu_load_management_task(void *pvParameters);

// NEW: Timer-based interrupt callbacks
static void flight_control_timer_callback(TimerHandle_t xTimer);
static void imu_timer_callback(TimerHandle_t xTimer);
static void command_timer_callback(TimerHandle_t xTimer);

// NEW: Advanced feature functions
static void gps_update_rth_data(void);
static bool gps_navigate_to_home(attitude_t *attitude, command_t *rth_command);
static void thermal_apply_imu_compensation(imu_data_t *imu_data);
static void thermal_check_m4_temperature(void);
static void motor_health_monitor(void);
static void motor_failure_compensation(motor_outputs_t *outputs);
static void cpu_load_balance_tasks(void);
static void m4_handle_m0_overflow_tasks(void);

// ==================== INITIALIZATION FUNCTIONS ====================

void drivers_init(void) {
    log_info("Initializing enhanced hardware drivers...");
    
    // Initialize communication buses
    if (spi_bus_init() != 0) {
        log_error("SPI bus initialization failed");
        system_state = SYSTEM_ERROR;
        return;
    }
    
    if (i2c_bus_init() != 0) {
        log_error("I2C bus initialization failed"); 
        system_state = SYSTEM_ERROR;
        return;
    }
    
    // Initialize IMU sensors with temperature monitoring
    imu_kx122_init();
    imu_lsm6ds3_init();
    imu_lsm303c_init();
    
    // Initialize RF communication
    if (rf_s2lpqtr_init() != 0) {
        log_warning("RF initialization failed - continuing without RF");
    }
    
    // Initialize motor control system
    motor_control_init();
    
    // Initialize GPS for RTH functionality
    gps_uart_init();
    
    // Initialize flash and USB
    flash_init();
    usb_bridge_init();
    
    // NEW: Initialize temperature monitoring ADC
    Chip_ADC_Init(LPC_ADC0);
    Chip_ADC_SetSampleRate(LPC_ADC0, 100000); // 100kHz sample rate
    Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH0, ENABLE); // M4 temperature channel
    
    log_info("Enhanced hardware drivers initialized successfully");
}

void middleware_init(void) {
    log_info("Initializing enhanced middleware components...");
    
    // Create mutexes for thread-safe data access
    imu_data_mutex = xSemaphoreCreateMutex();
    command_mutex = xSemaphoreCreateMutex();
    attitude_mutex = xSemaphoreCreateMutex();
    gps_data_mutex = xSemaphoreCreateMutex();       // NEW
    thermal_data_mutex = xSemaphoreCreateMutex();   // NEW
    
    if (imu_data_mutex == NULL || command_mutex == NULL || attitude_mutex == NULL ||
        gps_data_mutex == NULL || thermal_data_mutex == NULL) {
        log_error("Failed to create mutexes");
        system_state = SYSTEM_ERROR;
        return;
    }
    
    // Initialize existing middleware
    attitude_estimator_init();
    command_processor_init();
    telemetry_init();
    safety_monitor_init();
    
    // NEW: Initialize motor health monitoring
    for (int i = 0; i < 4; i++) {
        motor_health.motor_healthy[i] = true;
        motor_health.motor_current[i] = 0.0f;
        motor_health.motor_failure_count[i] = 0;
    }
    motor_health.failed_motor_mask = 0;
    motor_health.thrust_loss_detected = false;
    
    log_info("Enhanced middleware initialized successfully");
}

void app_init(void) {
    log_info("Initializing enhanced application with timer-based control...");
    
    // NEW: Create timer-based control loops for precise timing
    flight_control_timer = xTimerCreate("FlightCtrlTimer", pdMS_TO_TICKS(1), // 1ms = 1kHz
                                       pdTRUE, NULL, flight_control_timer_callback);
    
    imu_timer = xTimerCreate("IMUTimer", pdMS_TO_TICKS(2), // 2ms = 500Hz
                            pdTRUE, NULL, imu_timer_callback);
    
    command_timer = xTimerCreate("CommandTimer", pdMS_TO_TICKS(10), // 10ms = 100Hz
                                pdTRUE, NULL, command_timer_callback);
    
    if (flight_control_timer == NULL || imu_timer == NULL || command_timer == NULL) {
        log_error("Failed to create control timers");
        system_state = SYSTEM_ERROR;
        return;
    }
    
    // Create enhanced tasks
    xTaskCreate(imu_task, "IMU", 1536, NULL, 4, &imu_task_handle);
    xTaskCreate(command_task, "Commands", 1024, NULL, 3, &command_task_handle);
    xTaskCreate(telemetry_task, "Telemetry", 1024, NULL, 2, &telemetry_task_handle);
    xTaskCreate(safety_task, "Safety", 1024, NULL, 6, NULL);
    xTaskCreate(m0_monitor_task, "M0Monitor", 512, NULL, 1, NULL);
    
    // NEW: Create additional tasks for enhanced functionality
    xTaskCreate(gps_task, "GPS_RTH", 1024, NULL, 2, &gps_task_handle);
    xTaskCreate(thermal_monitoring_task, "Thermal", 768, NULL, 3, &thermal_task_handle);
    xTaskCreate(cpu_load_management_task, "CPULoad", 512, NULL, 1, NULL);
    
    // Start timer-based control loops
    if (xTimerStart(flight_control_timer, 0) != pdPASS) {
        log_error("Failed to start flight control timer");
        system_state = SYSTEM_ERROR;
        return;
    }
    
    if (xTimerStart(imu_timer, 0) != pdPASS) {
        log_error("Failed to start IMU timer");
        system_state = SYSTEM_ERROR;
        return;
    }
    
    if (xTimerStart(command_timer, 0) != pdPASS) {
        log_error("Failed to start command timer");
        system_state = SYSTEM_ERROR;
        return;
    }
    
    system_state = SYSTEM_READY;
    log_info("Enhanced application initialized - Timer-based control active");
}

void app_main_loop(void) {
    // Main loop now focuses on low-priority tasks and M0 coordination
    m0_watchdog_kick();
    
    // NEW: Check if M4 needs to handle M0 overflow tasks
    uint16_t m0_load = m0_get_cpu_load_estimate();
    if (m0_load > 85) { // M0 overloaded
        m4_handle_m0_overflow_tasks();
        cpu_load.tasks_offloaded_to_m4++;
    }
    
    // Check M0 status
    uint32_t m0_status = m0_get_status();
    if (m0_status != 0) {
        log_warning("M0 status flags: 0x%08X", m0_status);
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz background loop
}

// ==================== TIMER-BASED CONTROL LOOPS ====================

static void flight_control_timer_callback(TimerHandle_t xTimer) {
    (void)xTimer;
    
    if (system_state != SYSTEM_READY && system_state != SYSTEM_RTH_MODE) {
        return;
    }
    
    // High-precision flight control executed from timer interrupt
    attitude_t attitude;
    command_t command;
    
    // Get data with minimal blocking
    if (xSemaphoreTakeFromISR(attitude_mutex, NULL) == pdTRUE) {
        attitude = current_attitude;
        xSemaphoreGiveFromISR(attitude_mutex, NULL);
    } else {
        return; // Skip this cycle
    }
    
    if (xSemaphoreTakeFromISR(command_mutex, NULL) == pdTRUE) {
        command = current_command;
        xSemaphoreGiveFromISR(command_mutex, NULL);
    }
    
    // NEW: Handle RTH mode
    if (current_flight_mode == FLIGHT_MODE_RTH) {
        if (gps_navigate_to_home(&attitude, &command)) {
            // RTH navigation successful
            system_state = SYSTEM_RTH_MODE;
        }
    }
    
    // Run flight controller
    switch (current_flight_mode) {
        case FLIGHT_MODE_DISARMED:
            motor_outputs.motor1 = motor_outputs.motor2 = 
            motor_outputs.motor3 = motor_outputs.motor4 = 0;
            break;
            
        case FLIGHT_MODE_STABILIZE:
        case FLIGHT_MODE_RTH:
            flight_controller_stabilize(&attitude, &command, &motor_outputs);
            break;
            
        case FLIGHT_MODE_ALTITUDE_HOLD:
            flight_controller_altitude_hold(&attitude, &command, &motor_outputs);
            break;
            
        case FLIGHT_MODE_AUTONOMOUS:
            flight_controller_autonomous(&attitude, &command, &motor_outputs);
            break;
            
        case FLIGHT_MODE_MOTOR_FAILURE:
            flight_controller_stabilize(&attitude, &command, &motor_outputs);
            motor_failure_compensation(&motor_outputs); // NEW: Compensate for failed motor
            break;
    }
    
    // NEW: Check for motor failures and thermal limits before applying outputs
    motor_health_monitor();
    
    if (!thermal_data.thermal_critical) {
        motor_control_set_outputs(&motor_outputs);
    } else {
        // Thermal protection - reduce power
        motor_outputs_t reduced_outputs = motor_outputs;
        reduced_outputs.motor1 = (reduced_outputs.motor1 * 70) / 100;
        reduced_outputs.motor2 = (reduced_outputs.motor2 * 70) / 100;
        reduced_outputs.motor3 = (reduced_outputs.motor3 * 70) / 100;
        reduced_outputs.motor4 = (reduced_outputs.motor4 * 70) / 100;
        motor_control_set_outputs(&reduced_outputs);
    }
}

static void imu_timer_callback(TimerHandle_t xTimer) {
    (void)xTimer;
    
    if (system_state == SYSTEM_THERMAL_SHUTDOWN) {
        return;
    }
    
    // Precise IMU reading from timer interrupt
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Notify IMU task to read data
    vTaskNotifyGiveFromISR(imu_task_handle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void command_timer_callback(TimerHandle_t xTimer) {
    (void)xTimer;
    
    // Notify command task to check for new commands
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(command_task_handle, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ==================== ENHANCED TASK IMPLEMENTATIONS ====================

static void imu_task(void *pvParameters) {
    (void)pvParameters;
    float accel[3], gyro[3], mag[3], temp;
    
    while (1) {
        // Wait for timer notification for precise timing
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Read all IMU sensors
        imu_kx122_read(accel, &temp);
        imu_lsm6ds3_read(gyro, &temp);
        imu_lsm303c_read(mag, &temp);
        
        // Update IMU data with temperature compensation
        if (xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            current_imu_data.accel_x = accel[0];
            current_imu_data.accel_y = accel[1];
            current_imu_data.accel_z = accel[2];
            current_imu_data.gyro_x = gyro[0];
            current_imu_data.gyro_y = gyro[1];
            current_imu_data.gyro_z = gyro[2];
            current_imu_data.mag_x = mag[0];
            current_imu_data.mag_y = mag[1];
            current_imu_data.mag_z = mag[2];
            current_imu_data.temperature = temp;
            current_imu_data.timestamp = xTaskGetTickCount();
            
            // NEW: Apply temperature compensation
            thermal_apply_imu_compensation(&current_imu_data);
            
            xSemaphoreGive(imu_data_mutex);
        }
        
        // Run attitude estimation
        attitude_t new_attitude;
        attitude_estimator_update(&current_imu_data, &new_attitude);
        
        if (xSemaphoreTake(attitude_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            current_attitude = new_attitude;
            xSemaphoreGive(attitude_mutex);
        }
    }
}

static void command_task(void *pvParameters) {
    (void)pvParameters;
    uint8_t rf_buffer[64];
    
    while (1) {
        // Wait for timer notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Check for RF commands
        int bytes_received = rf_receive_hamming(rf_buffer, sizeof(rf_buffer));
        if (bytes_received > 0) {
            command_t new_command;
            if (command_processor_parse(rf_buffer, bytes_received, &new_command) == 0) {
                if (xSemaphoreTake(command_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    current_command = new_command;
                    
                    if (new_command.flight_mode != current_flight_mode) {
                        log_info("Flight mode changed from %d to %d", 
                               current_flight_mode, new_command.flight_mode);
                        current_flight_mode = new_command.flight_mode;
                    }
                    
                    xSemaphoreGive(command_mutex);
                }
            }
        }
        
        // NEW: Check for RF timeout and initiate RTH if necessary
        if (command_processor_is_timeout() && current_flight_mode != FLIGHT_MODE_DISARMED) {
            log_warning("RF timeout - initiating Return-to-Home");
            if (gps_data.home_set && gps_data.gps_valid) {
                current_flight_mode = FLIGHT_MODE_RTH;
                system_state = SYSTEM_RTH_MODE;
            } else {
                log_error("RTH not available - no GPS home position");
                current_flight_mode = FLIGHT_MODE_DISARMED;
            }
        }
    }
}

// NEW: GPS task for RTH functionality
static void gps_task(void *pvParameters) {
    (void)pvParameters;
    char nmea_buffer[128];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5Hz GPS update
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Read GPS data
        if (gps_uart_read_line(nmea_buffer, sizeof(nmea_buffer)) > 0) {
            float lat, lon;
            if (gps_uart_parse_gga(nmea_buffer, &lat, &lon) == 0) {
                if (xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    gps_data.latitude = lat;
                    gps_data.longitude = lon;
                    gps_data.satellites = 8; // TODO: Parse from NMEA
                    gps_data.fix_quality = 1; // TODO: Parse from NMEA
                    gps_data.gps_valid = (gps_data.satellites >= 4);
                    
                    // Set home position on first good GPS fix when disarmed
                    if (!gps_data.home_set && gps_data.gps_valid && 
                        current_flight_mode == FLIGHT_MODE_DISARMED) {
                        gps_data.home_lat = lat;
                        gps_data.home_lon = lon;
                        gps_data.home_set = true;
                        log_info("Home position set: %.6f, %.6f", lat, lon);
                    }
                    
                    // Update RTH navigation data
                    gps_update_rth_data();
                    
                    xSemaphoreGive(gps_data_mutex);
                }
            }
        }
    }
}

// NEW: Thermal monitoring task
static void thermal_monitoring_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1Hz thermal check
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if (xSemaphoreTake(thermal_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Check M4 core temperature
            thermal_check_m4_temperature();
            
            // Get IMU temperatures
            thermal_data.imu_temp[0] = current_imu_data.temperature; // From last IMU reading
            
            // Check thermal limits
            thermal_data.thermal_warning = (thermal_data.m4_core_temp > 75.0f);
            thermal_data.thermal_critical = (thermal_data.m4_core_temp > 85.0f);
            
            if (thermal_data.thermal_critical) {
                log_error("CRITICAL: M4 temperature %.1f°C - initiating thermal shutdown", 
                         thermal_data.m4_core_temp);
                system_state = SYSTEM_THERMAL_SHUTDOWN;
                current_flight_mode = FLIGHT_MODE_DISARMED;
            } else if (thermal_data.thermal_warning) {
                log_warning("M4 temperature high: %.1f°C", thermal_data.m4_core_temp);
            }
            
            thermal_data.last_temp_check = xTaskGetTickCount();
            xSemaphoreGive(thermal_data_mutex);
        }
    }
}

// NEW: CPU load management task
static void cpu_load_management_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz load management
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Balance CPU load between M4 and M0
        cpu_load_balance_tasks();
        
        // Update load statistics
        cpu_load.m4_cpu_load = 50; // TODO: Calculate actual M4 CPU usage
        cpu_load.m0_cpu_load = m0_get_cpu_load_estimate();
        cpu_load.load_critical = (cpu_load.m4_cpu_load > 90 || cpu_load.m0_cpu_load > 90);
        cpu_load.last_load_check = xTaskGetTickCount();
        
        if (cpu_load.load_critical) {
            log_warning("CPU load critical: M4=%d%% M0=%d%%", 
                       cpu_load.m4_cpu_load, cpu_load.m0_cpu_load);
        }
    }
}

// Continue with other existing task implementations...
static void telemetry_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if (system_state == SYSTEM_READY || system_state == SYSTEM_RTH_MODE) {
            telemetry_data_t telem_data;
            
            // Gather telemetry including new GPS and thermal data
            if (xSemaphoreTake(attitude_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                telem_data.roll = current_attitude.roll;
                telem_data.pitch = current_attitude.pitch;
                telem_data.yaw = current_attitude.yaw;
                telem_data.roll_rate = current_attitude.roll_rate;
                telem_data.pitch_rate = current_attitude.pitch_rate;
                telem_data.yaw_rate = current_attitude.yaw_rate;
                xSemaphoreGive(attitude_mutex);
            }
            
            if (xSemaphoreTake(gps_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                telem_data.gps_latitude = gps_data.latitude;
                telem_data.gps_longitude = gps_data.longitude;
                telem_data.gps_satellites = gps_data.satellites;
                telem_data.distance_to_home = gps_data.distance_to_home;
                xSemaphoreGive(gps_data_mutex);
            }
            
            if (xSemaphoreTake(thermal_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                telem_data.m4_core_temp = thermal_data.m4_core_temp;
                telem_data.thermal_warning = thermal_data.thermal_warning;
                xSemaphoreGive(thermal_data_mutex);
            }
            
            // Motor health data
            telem_data.motor_health_mask = motor_health.failed_motor_mask;
            telem_data.thrust_loss_detected = motor_health.thrust_loss_detected;
            
            // System status
            telem_data.system_status = system_state;
            telem_data.flight_mode = current_flight_mode;
            telem_data.cpu_load_m4 = cpu_load.m4_cpu_load;
            telem_data.cpu_load_m0 = cpu_load.m0_cpu_load;
            
            telemetry_transmit(&telem_data);
        }
    }
}

static void safety_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Enhanced safety monitoring with new failure modes
        uint32_t m0_errors = m0_error_get();
        if (m0_errors != 0) {
            log_error("M0 error flags: 0x%08X", m0_errors);
            
            if (m0_errors & (0x01 | 0x02 | 0x10)) {
                current_flight_mode = FLIGHT_MODE_DISARMED;
                system_state = SYSTEM_ERROR;
            }
        }
        
        // Check for motor failures
        if (motor_health.failed_motor_mask != 0) {
            log_warning("Motor failure detected: mask 0x%02X", motor_health.failed_motor_mask);
            if (__builtin_popcount(motor_health.failed_motor_mask) >= 2) {
                // Two or more motors failed - emergency landing
                current_flight_mode = FLIGHT_MODE_DISARMED;
                log_error("Multiple motor failures - emergency disarm");
            } else if (current_flight_mode != FLIGHT_MODE_DISARMED) {
                // Single motor failure - switch to compensation mode
                current_flight_mode = FLIGHT_MODE_MOTOR_FAILURE;
                log_info("Switching to motor failure compensation mode");
            }
        }
        
        // Check thermal status
        if (system_state == SYSTEM_THERMAL_SHUTDOWN) {
            current_flight_mode = FLIGHT_MODE_DISARMED;
            motor_control_disarm();
        }
        
        // Run other safety checks
        safety_status_t safety_status = safety_monitor_check();
        if (safety_status != SAFETY_OK) {
            log_error("Safety failure: %d", safety_status);
            current_flight_mode = FLIGHT_MODE_DISARMED;
        }
    }
}

static void m0_monitor_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5Hz
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        m0_check_all_hardware();
        m0_selftest_run();
        
        // Monitor task priorities
        UBaseType_t flight_priority = uxTaskPriorityGet(flight_control_task_handle);
        if (flight_priority && flight_priority < 5) {
            vTaskPrioritySet(flight_control_task_handle, 5);
            log_info("Restored flight control task priority");
        }
    }
}

// ==================== NEW FEATURE IMPLEMENTATIONS ====================

static void gps_update_rth_data(void) {
    if (!gps_data.home_set || !gps_data.gps_valid) {
        return;
    }
    
    // Calculate distance and bearing to home
    double lat1 = gps_data.home_lat * M_PI / 180.0;
    double lon1 = gps_data.home_lon * M_PI / 180.0;
    double lat2 = gps_data.latitude * M_PI / 180.0;
    double lon2 = gps_data.longitude * M_PI / 180.0;
    
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    
    double a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    gps_data.distance_to_home = 6371000 * c; // Distance in meters
    
    gps_data.bearing_to_home = atan2(sin(dlon) * cos(lat2), 
                                    cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon));
    gps_data.bearing_to_home = fmod(gps_data.bearing_to_home * 180.0 / M_PI + 360.0, 360.0);
}

static bool gps_navigate_to_home(attitude_t *attitude, command_t *rth_command) {
    if (!gps_data.home_set || !gps_data.gps_valid) {
        return false;
    }
    
    // Simple RTH navigation - fly towards home position
    float desired_yaw = gps_data.bearing_to_home * M_PI / 180.0f;
    float yaw_error = desired_yaw - attitude->yaw;
    
    // Wrap yaw error to [-π, π]
    while (yaw_error > M_PI) yaw_error -= 2*M_PI;
    while (yaw_error < -M_PI) yaw_error += 2*M_PI;
    
    // Set RTH command
    rth_command->throttle = 0.6f; // Moderate altitude hold
    rth_command->roll_cmd = 0.0f; // Level flight
    rth_command->pitch_cmd = (gps_data.distance_to_home > 5.0f) ? 0.1f : 0.0f; // Forward if not at home
    rth_command->yaw_cmd = yaw_error * 0.5f; // Turn towards home
    rth_command->flight_mode = FLIGHT_MODE_RTH;
    
    // Check if we've reached home (within 3 meters)
    if (gps_data.distance_to_home < 3.0f) {
        log_info("RTH completed - landing");
        rth_command->throttle = 0.1f; // Initiate landing
        return true;
    }
    
    return true;
}

static void thermal_apply_imu_compensation(imu_data_t *imu_data) {
    float temp_delta = imu_data->temperature - 25.0f; // Reference temperature
    
    // Apply temperature compensation to accelerometer
    imu_data->accel_x += temp_compensation_accel[0][0] * temp_delta + temp_compensation_accel[0][1];
    imu_data->accel_y += temp_compensation_accel[1][0] * temp_delta + temp_compensation_accel[1][1];
    imu_data->accel_z += temp_compensation_accel[2][0] * temp_delta + temp_compensation_accel[2][1];
    
    // Apply temperature compensation to gyroscope
    imu_data->gyro_x += temp_compensation_gyro[0][0] * temp_delta + temp_compensation_gyro[0][1];
    imu_data->gyro_y += temp_compensation_gyro[1][0] * temp_delta + temp_compensation_gyro[1][1];
    imu_data->gyro_z += temp_compensation_gyro[2][0] * temp_delta + temp_compensation_gyro[2][1];
}

static void thermal_check_m4_temperature(void) {
    // Read M4 core temperature from ADC
    Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
    while (!Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH0, ADC_DR_DONE_STAT)) {}
    
    uint32_t adc_value;
    Chip_ADC_ReadValue(LPC_ADC0, ADC_CH0, &adc_value);
    
    // Convert ADC reading to temperature (simplified)
    thermal_data.m4_core_temp = 25.0f + (adc_value - 2048) * 0.1f;
}

static void motor_health_monitor(void) {
    // Monitor motor health by checking current draw and response
    for (int i = 0; i < 4; i++) {
        uint16_t motor_pwm = (&motor_outputs.motor1)[i];
        
        // Estimate current based on PWM (simplified model)
        motor_health.motor_current[i] = (motor_pwm - MOTOR_MIN_PWM) * 0.01f;
        
        // Check for abnormal current draw
        if (motor_pwm > MOTOR_IDLE_PWM && motor_health.motor_current[i] < 0.1f) {
            // Motor commanded but no current draw - possible failure
            motor_health.motor_failure_count[i]++;
            
            if (motor_health.motor_failure_count[i] > 10) {
                motor_health.motor_healthy[i] = false;
                motor_health.failed_motor_mask |= (1 << i);
                log_error("Motor %d failure detected", i+1);
            }
        } else if (motor_health.motor_failure_count[i] > 0) {
            motor_health.motor_failure_count[i]--;
        }
        
        // Check for excessive current (overload)
        if (motor_health.motor_current[i] > 5.0f) {
            motor_health.motor_healthy[i] = false;
            motor_health.failed_motor_mask |= (1 << i);
            log_error("Motor %d overcurrent detected", i+1);
        }
    }
    
    // Detect thrust loss by checking attitude response
    static float last_roll = 0, last_pitch = 0;
    float roll_rate_change = fabsf(current_attitude.roll_rate - last_roll);
    float pitch_rate_change = fabsf(current_attitude.pitch_rate - last_pitch);
    
    if ((motor_outputs.motor1 + motor_outputs.motor2 + motor_outputs.motor3 + motor_outputs.motor4) > 
        (4 * MOTOR_IDLE_PWM) && (roll_rate_change < 0.01f && pitch_rate_change < 0.01f)) {
        // Motors spinning but no attitude response - possible thrust loss
        static uint32_t thrust_loss_count = 0;
        thrust_loss_count++;
        
        if (thrust_loss_count > 100) { // 100ms at 1kHz
            motor_health.thrust_loss_detected = true;
            log_error("Thrust loss detected");
        }
    }
    
    last_roll = current_attitude.roll_rate;
    last_pitch = current_attitude.pitch_rate;
}

static void motor_failure_compensation(motor_outputs_t *outputs) {
    // Compensate for failed motors by redistributing thrust
    if (motor_health.failed_motor_mask == 0) {
        return; // No failures
    }
    
    uint8_t healthy_count = 4 - __builtin_popcount(motor_health.failed_motor_mask);
    if (healthy_count < 3) {
        // Too many failures for compensation
        outputs->motor1 = outputs->motor2 = outputs->motor3 = outputs->motor4 = 0;
        return;
    }
    
    // Zero out failed motors
    if (!(motor_health.failed_motor_mask & 0x01)) outputs->motor1 = 0;
    if (!(motor_health.failed_motor_mask & 0x02)) outputs->motor2 = 0;
    if (!(motor_health.failed_motor_mask & 0x04)) outputs->motor3 = 0;
    if (!(motor_health.failed_motor_mask & 0x08)) outputs->motor4 = 0;
    
    // Redistribute thrust to remaining motors
    float compensation_factor = 4.0f / healthy_count;
    
    if (motor_health.motor_healthy[0]) outputs->motor1 *= compensation_factor;
    if (motor_health.motor_healthy[1]) outputs->motor2 *= compensation_factor;
    if (motor_health.motor_healthy[2]) outputs->motor3 *= compensation_factor;
    if (motor_health.motor_healthy[3]) outputs->motor4 *= compensation_factor;
    
    // Limit to maximum PWM
    if (outputs->motor1 > MOTOR_MAX_PWM) outputs->motor1 = MOTOR_MAX_PWM;
    if (outputs->motor2 > MOTOR_MAX_PWM) outputs->motor2 = MOTOR_MAX_PWM;
    if (outputs->motor3 > MOTOR_MAX_PWM) outputs->motor3 = MOTOR_MAX_PWM;
    if (outputs->motor4 > MOTOR_MAX_PWM) outputs->motor4 = MOTOR_MAX_PWM;
}

static void cpu_load_balance_tasks(void) {
    // Balance CPU load between M4 and M0
    uint16_t m0_load = m0_get_cpu_load_estimate();
    
    if (m0_load > 80) {
        // M0 overloaded - have M4 take over some tasks
        log_info("M0 overloaded (%d%%) - M4 taking over tasks", m0_load);
        m4_handle_m0_overflow_tasks();
    }
}

static void m4_handle_m0_overflow_tasks(void) {
    // M4 handles overflow tasks from M0
    static uint32_t last_overflow_handle = 0;
    uint32_t current_time = xTaskGetTickCount();
    
    if (current_time - last_overflow_handle > pdMS_TO_TICKS(100)) {
        // Handle battery monitoring on M4
        thermal_check_m4_temperature();
        
        // Handle GPS parsing on M4 (already done in GPS task)
        
        // Handle performance counter updates
        cpu_load.tasks_offloaded_to_m4++;
        
        last_overflow_handle = current_time;
    }
}