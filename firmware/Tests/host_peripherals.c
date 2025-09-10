#include "host_peripherals.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>

/* Register definitions for sensors */
// KX122 Registers
#define KX122_WHO_AM_I      0x0F
#define KX122_REG_CTRL1     0x1B
#define KX122_REG_CTRL2     0x18
#define KX122_REG_CTRL3     0x1C
#define KX122_REG_OUTX_L    0x06
#define KX122_REG_OUTX_H    0x07
#define KX122_REG_OUTY_L    0x08
#define KX122_REG_OUTY_H    0x09
#define KX122_REG_OUTZ_L    0x0A
#define KX122_REG_OUTZ_H    0x0B

// LSM6DS3 Registers
#define LSM6DS3_WHO_AM_I    0x0F
#define LSM6DS3_CTRL1_XL    0x10
#define LSM6DS3_CTRL2_G     0x11
#define LSM6DS3_CTRL3_C     0x12
#define LSM6DS3_OUTX_L_XL   0x28
#define LSM6DS3_OUTX_H_XL   0x29
#define LSM6DS3_OUTY_L_XL   0x2A
#define LSM6DS3_OUTY_H_XL   0x2B
#define LSM6DS3_OUTZ_L_XL   0x2C
#define LSM6DS3_OUTZ_H_XL   0x2D
#define LSM6DS3_OUTX_L_G    0x22
#define LSM6DS3_OUTX_H_G    0x23
#define LSM6DS3_OUTY_L_G    0x24
#define LSM6DS3_OUTY_H_G    0x25
#define LSM6DS3_OUTZ_L_G    0x26
#define LSM6DS3_OUTZ_H_G    0x27

// LSM303C Registers
#define LSM303C_WHO_AM_I    0x0F
#define LSM303C_CTRL_REG1_A 0x20
#define LSM303C_CTRL_REG2_A 0x21
#define LSM303C_CTRL_REG3_A 0x22
#define LSM303C_OUT_X_L_A   0x28
#define LSM303C_OUT_X_H_A   0x29
#define LSM303C_OUT_Y_L_A   0x2A
#define LSM303C_OUT_Y_H_A   0x2B
#define LSM303C_OUT_Z_L_A   0x2C
#define LSM303C_OUT_Z_H_A   0x2D

static int gpio_state[256];
static int power_state[16];

/* Simulated sensor device structure */
// Orientation structure for sensor simulation
typedef struct {
    float roll;    // Roll angle in degrees
    float pitch;   // Pitch angle in degrees
    float yaw;     // Yaw angle in degrees
    float acc_x;   // Linear acceleration in m/s^2
    float acc_y;
    float acc_z;
    float gyro_x;  // Angular velocity in deg/s
    float gyro_y;
    float gyro_z;
    float mag_x;   // Magnetic field in uT
    float mag_y;
    float mag_z;
} orientation_t;

typedef struct {
    uint8_t regs[256];
    uint8_t who_am_i;
    int enabled;             // Power state
    int odr;                // Output data rate
    int range;              // Measurement range
    int16_t accel[3];      // Current accelerometer values
    int16_t gyro[3];       // Current gyroscope values (LSM6DS3 only)
    int16_t mag[3];        // Current magnetometer values (LSM303C only)
    orientation_t orientation; // Current orientation data
    float update_rate;      // Update rate in Hz
    uint64_t last_update;   // Last update timestamp
} sensor_device_t;

/* Sensor device instances */
static sensor_device_t kx122 = {
    .who_am_i = 0x1B    // KX122 WHO_AM_I value
};

static sensor_device_t lsm6ds3 = {
    .who_am_i = 0x69    // LSM6DS3 WHO_AM_I value
};

static sensor_device_t lsm303c = {
    .who_am_i = 0x41    // LSM303C WHO_AM_I value
};

/* Simple I2C register store for devices keyed by 7-bit address */
typedef struct {
    uint8_t regs[256];
} i2c_device_t;

/* small fixed table for common sensors used on the board */
static i2c_device_t i2c_devices[128];

/* chip host GPIO port simulation */
#include "chip_host.h"
LPC_GPIO_PORT_Type _gpio_port;
LPC_GPIO_PORT_Type *LPC_GPIO_PORT = &_gpio_port;

#include <math.h>
#define PI 3.14159265358979323846f

// Convert orientation to sensor readings
static void orientation_to_sensor_readings(orientation_t *orientation, sensor_device_t *sensor) {
    // Convert angles to radians
    float roll_rad = orientation->roll * PI / 180.0f;
    float pitch_rad = orientation->pitch * PI / 180.0f;
    float yaw_rad = orientation->yaw * PI / 180.0f;

    // Gravity vector (9.81 m/s^2)
    const float g = 9.81f;

    // Calculate gravity components based on orientation
    float sin_roll = sinf(roll_rad);
    float cos_roll = cosf(roll_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_pitch = cosf(pitch_rad);

    // Calculate accelerometer readings (including gravity)
    float acc_x = orientation->acc_x + g * sin_pitch;
    float acc_y = orientation->acc_y - g * sin_roll * cos_pitch;
    float acc_z = orientation->acc_z - g * cos_roll * cos_pitch;

    // Convert to sensor units (assuming ±2g range = ±32768)
    sensor->accel[0] = (int16_t)(acc_x * 32768.0f / (2.0f * g));
    sensor->accel[1] = (int16_t)(acc_y * 32768.0f / (2.0f * g));
    sensor->accel[2] = (int16_t)(acc_z * 32768.0f / (2.0f * g));

    // Gyroscope readings (assuming ±250 dps = ±32768)
    if (sensor->gyro) {
        sensor->gyro[0] = (int16_t)(orientation->gyro_x * 32768.0f / 250.0f);
        sensor->gyro[1] = (int16_t)(orientation->gyro_y * 32768.0f / 250.0f);
        sensor->gyro[2] = (int16_t)(orientation->gyro_z * 32768.0f / 250.0f);
    }

    // Magnetometer readings (simulating Earth's magnetic field ~50 µT)
    if (sensor->mag) {
        float mag_strength = 50.0f;
        // Simulate magnetic field aligned with North
        sensor->mag[0] = (int16_t)(mag_strength * cosf(yaw_rad) * 32768.0f / 50.0f);
        sensor->mag[1] = (int16_t)(mag_strength * sinf(yaw_rad) * 32768.0f / 50.0f);
        sensor->mag[2] = (int16_t)(mag_strength * sinf(pitch_rad) * 32768.0f / 50.0f);
    }
}

// Function to update simulated sensor data
static void update_sensor_data(sensor_device_t *sensor) {
    if (!sensor->enabled) return;

    // Get current timestamp in milliseconds
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t current_time = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

    // Check if it's time to update based on update_rate
    if (sensor->update_rate > 0) {
        uint64_t update_interval = (uint64_t)(1000.0f / sensor->update_rate);
        if (current_time - sensor->last_update < update_interval) {
            return;
        }
    }
    sensor->last_update = current_time;

    // Add some noise to the orientation data
    float noise_amplitude = 0.1f;  // Small random variations
    sensor->orientation.acc_x += ((float)rand() / RAND_MAX - 0.5f) * noise_amplitude;
    sensor->orientation.acc_y += ((float)rand() / RAND_MAX - 0.5f) * noise_amplitude;
    sensor->orientation.acc_z += ((float)rand() / RAND_MAX - 0.5f) * noise_amplitude;

    // Convert orientation to sensor readings
    orientation_to_sensor_readings(&sensor->orientation, sensor);
}

int host_spi_init(void)
{
    /* Initialize default sensor registers and states */
    memset(&kx122, 0, sizeof(kx122));
    memset(&lsm6ds3, 0, sizeof(lsm6ds3));
    memset(&lsm303c, 0, sizeof(lsm303c));

    /* Set WHO_AM_I values */
    kx122.regs[KX122_WHO_AM_I] = kx122.who_am_i;
    lsm6ds3.regs[LSM6DS3_WHO_AM_I] = lsm6ds3.who_am_i;
    lsm303c.regs[LSM303C_WHO_AM_I] = lsm303c.who_am_i;

    /* Initialize I2C devices table */
    i2c_devices[0x1E].regs[0x0F] = lsm303c.who_am_i;  // LSM303C
    i2c_devices[0x6A].regs[0x0F] = lsm6ds3.who_am_i;  // LSM6DS3
    i2c_devices[0x1F].regs[0x0F] = kx122.who_am_i;    // KX122 (if using I2C mode)

    srand(time(NULL));  // Initialize random number generator for sensor noise
    return 0;
}

int host_spi_transfer(const uint8_t *tx, uint8_t *rx, int len)
{
    if (!tx || !rx || len <= 0) return -1;

    uint8_t reg = tx[0];
    sensor_device_t *current_sensor = NULL;

    // Determine which sensor is being accessed based on CS pin state
    // This would need to be enhanced based on your CS pin mapping

    // Example sensor selection (modify based on your CS pin configuration)
    if (host_gpio_get(0) == 0) current_sensor = &kx122;
    else if (host_gpio_get(1) == 0) current_sensor = &lsm6ds3;
    else if (host_gpio_get(2) == 0) current_sensor = &lsm303c;

    if (!current_sensor) {
        // No sensor selected, return error pattern
        for (int i = 0; i < len; i++) rx[i] = 0xFF;
        return 0;
    }

    // Update simulated sensor data
    update_sensor_data(current_sensor);

    // Handle read/write
    if (reg & 0x80) {  // Read operation
        reg &= 0x7F;  // Clear read bit
        for (int i = 1; i < len; i++) {
            // Handle special registers that need real-time data
            switch (reg + i - 1) {
                case KX122_REG_OUTX_L:
                case LSM6DS3_OUTX_L_XL:
                case LSM303C_OUT_X_L_A:
                    rx[i] = current_sensor->accel[0] & 0xFF;
                    break;
                case KX122_REG_OUTX_H:
                case LSM6DS3_OUTX_H_XL:
                case LSM303C_OUT_X_H_A:
                    rx[i] = (current_sensor->accel[0] >> 8) & 0xFF;
                    break;
                // Add similar cases for Y and Z axes
                default:
                    rx[i] = current_sensor->regs[reg + i - 1];
            }
        }
    } else {  // Write operation
        for (int i = 1; i < len; i++) {
            current_sensor->regs[reg + i - 1] = tx[i];
            
            // Handle special register writes
            switch (reg + i - 1) {
                case KX122_REG_CTRL1:
                    current_sensor->enabled = (tx[i] & 0x80) != 0;  // PC1 bit
                    break;
                case LSM6DS3_CTRL1_XL:
                    current_sensor->enabled = (tx[i] & 0x0F) != 0;  // ODR_XL bits
                    break;
                // Add more cases for other control registers
            }
        }
    }

    return 0;
}

void host_spi_cs_set(int port, int pin, int level)
{
    /* Map port/pin to a gpio index and set it */
    int idx = port * 32 + pin;
    host_gpio_set(idx, level ? 1 : 0);
}

int host_i2c_write(uint8_t addr, const uint8_t *data, int len)
{
    if (!data || len <= 0) return -1;

    sensor_device_t *current_sensor = NULL;
    
    // Select sensor based on I2C address
    switch (addr) {
        case 0x1E: current_sensor = &lsm303c; break;
        case 0x6A: current_sensor = &lsm6ds3; break;
        case 0x1F: current_sensor = &kx122; break;
    }

    if (current_sensor) {
        uint8_t reg = data[0];
        for (int i = 1; i < len; i++) {
            current_sensor->regs[reg + i - 1] = data[i];
            
            // Handle special register writes
            switch (reg + i - 1) {
                case LSM6DS3_CTRL1_XL:
                    current_sensor->enabled = (data[i] & 0x0F) != 0;
                    break;
                case LSM303C_CTRL_REG1_A:
                    current_sensor->enabled = (data[i] & 0x07) != 0;
                    break;
                // Add more cases as needed
            }
        }
    }

    // Update I2C devices table for compatibility
    uint8_t reg = data[0];
    for (int i = 1; i < len; i++) {
        i2c_devices[addr].regs[reg + i - 1] = data[i];
    }
    
    return len - 1;
}

int host_i2c_read(uint8_t addr, uint8_t *data, int len)
{
    if (!data || len <= 0) return -1;

    sensor_device_t *current_sensor = NULL;
    
    // Select sensor based on I2C address
    switch (addr) {
        case 0x1E: current_sensor = &lsm303c; break;
        case 0x6A: current_sensor = &lsm6ds3; break;
        case 0x1F: current_sensor = &kx122; break;
    }

    if (current_sensor) {
        update_sensor_data(current_sensor);
        for (int i = 0; i < len; i++) {
            // Handle special registers that need real-time data
            switch (i) {
                case LSM6DS3_OUTX_L_XL:
                    data[i] = current_sensor->accel[0] & 0xFF;
                    break;
                case LSM6DS3_OUTX_H_XL:
                    data[i] = (current_sensor->accel[0] >> 8) & 0xFF;
                    break;
                // Add more cases for other sensor data registers
                default:
                    data[i] = current_sensor->regs[i];
            }
        }
    } else {
        // Fallback to simple I2C device table
        for (int i = 0; i < len; i++) {
            data[i] = i2c_devices[addr].regs[i];
        }
    }
    
    return len;
}

/* simple busy-wait implementation */
void Chip_Clock_System_BusyWait_ms(uint32_t ms) {
    host_delay_ms(ms);
}

void host_gpio_set(int pin, int value)
{
    if (pin < 0 || pin >= (int)(sizeof(gpio_state)/sizeof(gpio_state[0]))) return;
    gpio_state[pin] = value ? 1 : 0;
}

int host_gpio_get(int pin)
{
    if (pin < 0 || pin >= (int)(sizeof(gpio_state)/sizeof(gpio_state[0]))) return 0;
    return gpio_state[pin];
}

void host_power_enable(int rail_id, int enable)
{
    if (rail_id < 0 || rail_id >= (int)(sizeof(power_state)/sizeof(power_state[0]))) return;
    power_state[rail_id] = enable ? 1 : 0;
}

int host_power_status(int rail_id)
{
    if (rail_id < 0 || rail_id >= (int)(sizeof(power_state)/sizeof(power_state[0]))) return 0;
    return power_state[rail_id];
}

void host_delay_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

// Set sensor orientation
void host_set_sensor_orientation(uint8_t sensor_id, float roll, float pitch, float yaw) {
    sensor_device_t *sensor = NULL;
    
    switch(sensor_id) {
        case 0: sensor = &kx122; break;
        case 1: sensor = &lsm6ds3; break;
        case 2: sensor = &lsm303c; break;
        default: return;
    }
    
    sensor->orientation.roll = roll;
    sensor->orientation.pitch = pitch;
    sensor->orientation.yaw = yaw;
}

// Set sensor linear acceleration
void host_set_sensor_acceleration(uint8_t sensor_id, float ax, float ay, float az) {
    sensor_device_t *sensor = NULL;
    
    switch(sensor_id) {
        case 0: sensor = &kx122; break;
        case 1: sensor = &lsm6ds3; break;
        case 2: sensor = &lsm303c; break;
        default: return;
    }
    
    sensor->orientation.acc_x = ax;
    sensor->orientation.acc_y = ay;
    sensor->orientation.acc_z = az;
}

// Set sensor angular velocity
void host_set_sensor_angular_velocity(uint8_t sensor_id, float gx, float gy, float gz) {
    sensor_device_t *sensor = NULL;
    
    switch(sensor_id) {
        case 1: sensor = &lsm6ds3; break;  // Only LSM6DS3 has gyro
        default: return;
    }
    
    sensor->orientation.gyro_x = gx;
    sensor->orientation.gyro_y = gy;
    sensor->orientation.gyro_z = gz;
}

// Set sensor magnetic field
void host_set_sensor_magnetic_field(uint8_t sensor_id, float mx, float my, float mz) {
    sensor_device_t *sensor = NULL;
    
    switch(sensor_id) {
        case 2: sensor = &lsm303c; break;  // Only LSM303C has magnetometer
        default: return;
    }
    
    sensor->orientation.mag_x = mx;
    sensor->orientation.mag_y = my;
    sensor->orientation.mag_z = mz;
}

// Get current sensor orientation
void host_get_sensor_orientation(uint8_t sensor_id, float *roll, float *pitch, float *yaw) {
    sensor_device_t *sensor = NULL;
    
    switch(sensor_id) {
        case 0: sensor = &kx122; break;
        case 1: sensor = &lsm6ds3; break;
        case 2: sensor = &lsm303c; break;
        default: return;
    }
    
    if (roll) *roll = sensor->orientation.roll;
    if (pitch) *pitch = sensor->orientation.pitch;
    if (yaw) *yaw = sensor->orientation.yaw;
}

const char *host_peripherals_status(void)
{
    /* Return a small static summary for logging */
    static char buf[256];
    int total_powered = 0;
    for (int i = 0; i < (int)(sizeof(power_state)/sizeof(power_state[0])); ++i) total_powered += power_state[i];
    
    // Add orientation information for active sensors
    int n = snprintf(buf, sizeof(buf), "GPIOs set, powered rails=%d\n", total_powered);
    if (lsm6ds3.enabled) {
        n += snprintf(buf + n, sizeof(buf) - n, 
            "LSM6DS3: roll=%.1f pitch=%.1f yaw=%.1f\n",
            lsm6ds3.orientation.roll,
            lsm6ds3.orientation.pitch,
            lsm6ds3.orientation.yaw);
    }
    if (lsm303c.enabled) {
        n += snprintf(buf + n, sizeof(buf) - n,
            "LSM303C: mag=[%.1f,%.1f,%.1f]",
            lsm303c.orientation.mag_x,
            lsm303c.orientation.mag_y,
            lsm303c.orientation.mag_z);
    }
    return buf;
}
