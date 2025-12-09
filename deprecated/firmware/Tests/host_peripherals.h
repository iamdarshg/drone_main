#ifndef HOST_PERIPHERALS_H
#define HOST_PERIPHERALS_H

#include <stdint.h>

/* Host-side peripheral shim used for building and testing firmware as a
 * user-mode binary. This provides minimal SPI, GPIO, power management and
 * timing APIs that mimic the shape of the LPC4330 firmware APIs so the
 * code can be linked and exercised on host. This is intentionally a
 * behavioral shim and not a cycle-accurate emulator. */

/* SPI */
int host_spi_init(void);
/* Full-duplex transfer: tx buffer of len bytes sent, rx buffer populated with len bytes.
 * Returns 0 on success. */
int host_spi_transfer(const uint8_t *tx, uint8_t *rx, int len);

/* Chip-select for SPI devices (port/pin mapping used by drivers) */
void host_spi_cs_set(int port, int pin, int level);

/* I2C simulation: write/read to device with 7-bit address */
int host_i2c_write(uint8_t addr, const uint8_t *data, int len);
int host_i2c_read(uint8_t addr, uint8_t *data, int len);

/* GPIO */
void host_gpio_set(int pin, int value);
int host_gpio_get(int pin);

/* Power (simulate power delivery rails) */
void host_power_enable(int rail_id, int enable);
int host_power_status(int rail_id);

/* Sleep/delay */
void host_delay_ms(int ms);

/* Sensor simulation control */
void host_set_sensor_orientation(uint8_t sensor_id, float roll, float pitch, float yaw);
void host_set_sensor_acceleration(uint8_t sensor_id, float ax, float ay, float az);
void host_set_sensor_angular_velocity(uint8_t sensor_id, float gx, float gy, float gz);
void host_set_sensor_magnetic_field(uint8_t sensor_id, float mx, float my, float mz);
void host_get_sensor_orientation(uint8_t sensor_id, float *roll, float *pitch, float *yaw);

/* Sensor IDs */
#define SENSOR_KX122   0
#define SENSOR_LSM6DS3 1
#define SENSOR_LSM303C 2

/* Logging-friendly state inspection */
const char *host_peripherals_status(void);

#endif
