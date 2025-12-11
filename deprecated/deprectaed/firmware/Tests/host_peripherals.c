#include "host_peripherals.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#ifdef _WIN32
#include <windows.h>
#define _CRT_SECURE_NO_WARNINGS
#define snprintf _snprintf_s
#else
#include <unistd.h>
#include <time.h>
#endif

// Platform-independent safe snprintf
#ifdef _WIN32
#define SAFE_SNPRINTF(buf, size, format, ...) _snprintf_s(buf, size, _TRUNCATE, format, __VA_ARGS__)
#else
#define SAFE_SNPRINTF(buf, size, format, ...) snprintf(buf, size, format, __VA_ARGS__)
#endif

// Sensor IDs (if not defined in host_peripherals.h)
#ifndef SENSOR_KX122
#define SENSOR_KX122   0
#define SENSOR_LSM6DS3 1
#define SENSOR_LSM303C 2
#endif

// Sensor register maps and simulation state
typedef struct {
    uint8_t regs[256];
    int enabled;
    struct {
        float roll;
        float pitch;
        float yaw;
        float acc[3];
        float gyro[3];
        float mag[3];
        float temp;
    } state;
} sensor_state_t;

// Static state
static uint8_t gpio_state[256];
static uint8_t power_state[16];
static sensor_state_t sensors[3];  // KX122, LSM6DS3, LSM303C

// Sensor register definitions
#define KX122_WHO_AM_I      0x0F
#define KX122_CTRL1         0x18
#define KX122_XOUT_L        0x06
#define KX122_XOUT_H        0x07
#define KX122_YOUT_L        0x08
#define KX122_YOUT_H        0x09
#define KX122_ZOUT_L        0x0A
#define KX122_ZOUT_H        0x0B

#define LSM6DS3_WHO_AM_I    0x0F
#define LSM6DS3_CTRL1_XL    0x10
#define LSM6DS3_CTRL2_G     0x11
#define LSM6DS3_STATUS_REG  0x1E
#define LSM6DS3_OUTX_L_XL   0x28
#define LSM6DS3_OUTX_H_XL   0x29
#define LSM6DS3_OUTY_L_XL   0x2A
#define LSM6DS3_OUTY_H_XL   0x2B
#define LSM6DS3_OUTZ_L_XL   0x2C
#define LSM6DS3_OUTZ_H_XL   0x2D

#define LSM303C_WHO_AM_I    0x0F
#define LSM303C_CTRL_REG1_A 0x20
#define LSM303C_STATUS_A    0x27
#define LSM303C_OUT_X_L_A   0x28
#define LSM303C_OUT_X_H_A   0x29
#define LSM303C_OUT_Y_L_A   0x2A
#define LSM303C_OUT_Y_H_A   0x2B
#define LSM303C_OUT_Z_L_A   0x2C
#define LSM303C_OUT_Z_H_A   0x2D

// Sensor characteristics
#define KX122_WHO_AM_I_VAL    0x1B
#define LSM6DS3_WHO_AM_I_VAL  0x69
#define LSM303C_WHO_AM_I_VAL  0x41

// Initialize all peripheral simulation
int host_spi_init(void) {
    // Clear all state
    memset(gpio_state, 0, sizeof(gpio_state));
    memset(power_state, 0, sizeof(power_state));
    memset(sensors, 0, sizeof(sensors));
    
    // Initialize sensor defaults
    sensors[SENSOR_KX122].regs[KX122_WHO_AM_I] = KX122_WHO_AM_I_VAL;
    sensors[SENSOR_LSM6DS3].regs[LSM6DS3_WHO_AM_I] = LSM6DS3_WHO_AM_I_VAL;
    sensors[SENSOR_LSM303C].regs[LSM303C_WHO_AM_I] = LSM303C_WHO_AM_I_VAL;
    
    return 0;
}

// Convert sensor orientation to accelerometer readings
static void update_sensor_readings(sensor_state_t *sensor) {
    if (!sensor->enabled) return;
    
    const float PI = 3.14159265358979323846f;
    const float G = 9.81f;
    
    // Convert angles to radians
    float roll_rad = sensor->state.roll * PI / 180.0f;
    float pitch_rad = sensor->state.pitch * PI / 180.0f;
    float yaw_rad = sensor->state.yaw * PI / 180.0f;
    
    // Calculate gravity components
    float sin_roll = sinf(roll_rad);
    float cos_roll = cosf(roll_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_pitch = cosf(pitch_rad);
    
    // Calculate accelerometer readings (including gravity)
    float acc_x = sensor->state.acc[0] + G * sin_pitch;
    float acc_y = sensor->state.acc[1] - G * sin_roll * cos_pitch;
    float acc_z = sensor->state.acc[2] - G * cos_roll * cos_pitch;
    
    // Convert to 16-bit signed integers and store in registers
    // Scale factor: ±2g range maps to ±32768
    const float ACC_SCALE = 32768.0f / (2.0f * G);
    int16_t raw_acc_x = (int16_t)(acc_x * ACC_SCALE);
    int16_t raw_acc_y = (int16_t)(acc_y * ACC_SCALE);
    int16_t raw_acc_z = (int16_t)(acc_z * ACC_SCALE);
    
    // Store in sensor registers based on sensor type
    if (sensor == &sensors[SENSOR_KX122]) {
        sensor->regs[KX122_XOUT_L] = raw_acc_x & 0xFF;
        sensor->regs[KX122_XOUT_H] = (raw_acc_x >> 8) & 0xFF;
        sensor->regs[KX122_YOUT_L] = raw_acc_y & 0xFF;
        sensor->regs[KX122_YOUT_H] = (raw_acc_y >> 8) & 0xFF;
        sensor->regs[KX122_ZOUT_L] = raw_acc_z & 0xFF;
        sensor->regs[KX122_ZOUT_H] = (raw_acc_z >> 8) & 0xFF;
    } else if (sensor == &sensors[SENSOR_LSM6DS3]) {
        sensor->regs[LSM6DS3_OUTX_L_XL] = raw_acc_x & 0xFF;
        sensor->regs[LSM6DS3_OUTX_H_XL] = (raw_acc_x >> 8) & 0xFF;
        sensor->regs[LSM6DS3_OUTY_L_XL] = raw_acc_y & 0xFF;
        sensor->regs[LSM6DS3_OUTY_H_XL] = (raw_acc_y >> 8) & 0xFF;
        sensor->regs[LSM6DS3_OUTZ_L_XL] = raw_acc_z & 0xFF;
        sensor->regs[LSM6DS3_OUTZ_H_XL] = (raw_acc_z >> 8) & 0xFF;
        
        // Update gyroscope readings (±250 dps maps to ±32768)
        const float GYRO_SCALE = 32768.0f / 250.0f;
        int16_t raw_gyro_x = (int16_t)(sensor->state.gyro[0] * GYRO_SCALE);
        int16_t raw_gyro_y = (int16_t)(sensor->state.gyro[1] * GYRO_SCALE);
        int16_t raw_gyro_z = (int16_t)(sensor->state.gyro[2] * GYRO_SCALE);
        
        sensor->regs[LSM6DS3_OUTX_L_G] = raw_gyro_x & 0xFF;
        sensor->regs[LSM6DS3_OUTX_H_G] = (raw_gyro_x >> 8) & 0xFF;
        sensor->regs[LSM6DS3_OUTY_L_G] = raw_gyro_y & 0xFF;
        sensor->regs[LSM6DS3_OUTY_H_G] = (raw_gyro_y >> 8) & 0xFF;
        sensor->regs[LSM6DS3_OUTZ_L_G] = raw_gyro_z & 0xFF;
        sensor->regs[LSM6DS3_OUTZ_H_G] = (raw_gyro_z >> 8) & 0xFF;
    }
    
    // Update magnetometer readings if applicable (LSM303C)
    if (sensor == &sensors[SENSOR_LSM303C]) {
        float mag_strength = 50.0f; // Earth's magnetic field ~50 µT
        sensor->state.mag[0] = mag_strength * cosf(yaw_rad) * cos_pitch;
        sensor->state.mag[1] = mag_strength * sinf(yaw_rad) * cos_pitch;
        sensor->state.mag[2] = mag_strength * sin_pitch;
    }
}

// SPI transfer implementation
int host_spi_transfer(const uint8_t *tx, uint8_t *rx, int len) {
    if (!tx || !rx || len <= 0) return -1;
    
    // Determine which sensor is selected based on GPIO state
    sensor_state_t *sensor = NULL;
    if (!gpio_state[0]) sensor = &sensors[SENSOR_KX122];
    else if (!gpio_state[1]) sensor = &sensors[SENSOR_LSM6DS3];
    else if (!gpio_state[2]) sensor = &sensors[SENSOR_LSM303C];
    
    if (!sensor) {
        memset(rx, 0xFF, len);  // No sensor selected
        return len;
    }
    
    // First byte is register address
    uint8_t reg = tx[0] & 0x7F;  // Mask out read bit
    bool is_read = (tx[0] & 0x80) != 0;
    
    if (is_read) {
        // Update sensor readings before read
        update_sensor_readings(sensor);
        
        // Copy register values to rx buffer
        for (int i = 1; i < len; i++) {
            rx[i] = sensor->regs[reg + i - 1];
        }
    } else {
        // Write values to registers
        for (int i = 1; i < len; i++) {
            sensor->regs[reg + i - 1] = tx[i];
            
            // Handle special register writes
            switch (reg + i - 1) {
                case KX122_CTRL1:
                    sensor->enabled = (tx[i] & 0x80) != 0;
                    break;
                case LSM6DS3_CTRL1_XL:
                    sensor->enabled = (tx[i] & 0x0F) != 0;
                    break;
                case LSM303C_CTRL_REG1_A:
                    sensor->enabled = (tx[i] & 0x07) != 0;
                    break;
            }
        }
    }
    
    return len;
}

void host_spi_cs_set(int port, int pin, int level) {
    if (pin >= 0 && pin < sizeof(gpio_state)) {
        gpio_state[pin] = level ? 1 : 0;
    }
}

int host_i2c_write(uint8_t addr, const uint8_t *data, int len) {
    if (!data || len <= 0) return -1;
    
    // Map I2C addresses to sensors
    sensor_state_t *sensor = NULL;
    switch (addr) {
        case 0x1E: sensor = &sensors[SENSOR_LSM303C]; break;
        case 0x6A: sensor = &sensors[SENSOR_LSM6DS3]; break;
        case 0x1F: sensor = &sensors[SENSOR_KX122]; break;
        default: return -1;
    }
    
    uint8_t reg = data[0];
    for (int i = 1; i < len; i++) {
        sensor->regs[reg + i - 1] = data[i];
    }
    
    return len - 1;
}

int host_i2c_read(uint8_t addr, uint8_t *data, int len) {
    if (!data || len <= 0) return -1;
    
    // Map I2C addresses to sensors
    sensor_state_t *sensor = NULL;
    switch (addr) {
        case 0x1E: sensor = &sensors[SENSOR_LSM303C]; break;
        case 0x6A: sensor = &sensors[SENSOR_LSM6DS3]; break;
        case 0x1F: sensor = &sensors[SENSOR_KX122]; break;
        default: return -1;
    }
    
    update_sensor_readings(sensor);
    memcpy(data, sensor->regs, len);
    
    return len;
}

void host_gpio_set(int pin, int value) {
    if (pin >= 0 && pin < sizeof(gpio_state)) {
        gpio_state[pin] = value ? 1 : 0;
    }
}

int host_gpio_get(int pin) {
    if (pin >= 0 && pin < sizeof(gpio_state)) {
        return gpio_state[pin];
    }
    return 0;
}

void host_power_enable(int rail_id, int enable) {
    if (rail_id >= 0 && rail_id < sizeof(power_state)) {
        power_state[rail_id] = enable ? 1 : 0;
    }
}

int host_power_status(int rail_id) {
    if (rail_id >= 0 && rail_id < sizeof(power_state)) {
        return power_state[rail_id];
    }
    return 0;
}

void host_delay_ms(int ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    struct timespec ts = {
        .tv_sec = ms / 1000,
        .tv_nsec = (ms % 1000) * 1000000L
    };
    nanosleep(&ts, NULL);
#endif
}

// New sensor orientation control functions
void host_set_sensor_orientation(uint8_t sensor_id, float roll, float pitch, float yaw) {
    if (sensor_id < 3) {
        sensors[sensor_id].state.roll = roll;
        sensors[sensor_id].state.pitch = pitch;
        sensors[sensor_id].state.yaw = yaw;
        update_sensor_readings(&sensors[sensor_id]);
    }
}

void host_set_sensor_acceleration(uint8_t sensor_id, float ax, float ay, float az) {
    if (sensor_id < 3) {
        sensors[sensor_id].state.acc[0] = ax;
        sensors[sensor_id].state.acc[1] = ay;
        sensors[sensor_id].state.acc[2] = az;
        update_sensor_readings(&sensors[sensor_id]);
    }
}

void host_set_sensor_angular_velocity(uint8_t sensor_id, float gx, float gy, float gz) {
    if (sensor_id == SENSOR_LSM6DS3) {
        sensors[sensor_id].state.gyro[0] = gx;
        sensors[sensor_id].state.gyro[1] = gy;
        sensors[sensor_id].state.gyro[2] = gz;
        update_sensor_readings(&sensors[sensor_id]);
    }
}

void host_set_sensor_magnetic_field(uint8_t sensor_id, float mx, float my, float mz) {
    if (sensor_id == SENSOR_LSM303C) {
        sensors[sensor_id].state.mag[0] = mx;
        sensors[sensor_id].state.mag[1] = my;
        sensors[sensor_id].state.mag[2] = mz;
        update_sensor_readings(&sensors[sensor_id]);
    }
}

void host_get_sensor_orientation(uint8_t sensor_id, float *roll, float *pitch, float *yaw) {
    if (sensor_id < 3) {
        if (roll) *roll = sensors[sensor_id].state.roll;
        if (pitch) *pitch = sensors[sensor_id].state.pitch;
        if (yaw) *yaw = sensors[sensor_id].state.yaw;
    }
}

const char *host_peripherals_status(void) {
    static char buf[512];
    size_t space_left = sizeof(buf);
    char *ptr = buf;
    int ret;
    
    // Count powered rails
    int powered = 0;
    for (size_t i = 0; i < sizeof(power_state); i++) {
        powered += power_state[i];
    }
    
    ret = SAFE_SNPRINTF(ptr, space_left, "Power rails: %d enabled\n", powered);
    if (ret < 0) return buf;
    ptr += ret;
    space_left -= ret;
    
    // Add sensor status
    const char *names[] = {"KX122", "LSM6DS3", "LSM303C"};
    for (int i = 0; i < 3 && space_left > 0; i++) {
        ret = SAFE_SNPRINTF(ptr, space_left,
            "%s: %s, Orientation: %.1f° %.1f° %.1f°\n",
            names[i],
            sensors[i].enabled ? "enabled" : "disabled",
            sensors[i].state.roll,
            sensors[i].state.pitch,
            sensors[i].state.yaw);
            
        if (ret < 0) break;
        ptr += ret;
        space_left -= ret;
    }
    
    return buf;
}
