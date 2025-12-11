#ifndef HOST_SENSOR_MODELS_H
#define HOST_SENSOR_MODELS_H

#include <stdint.h>

/* Simple sensor models for host-side simulation. Devices include LSM6DS3,
 * LSM303C, and KX122. Models expose a simple register space and support
 * SPI register read/write where the first byte is the address with bit 7
 * indicating read (1) or write (0). */

void host_sensor_models_init(void);
/* Load overrides for WHO_AM_I registers from a simple key=value file.
 * file format (per line): NAME=0xNN  (e.g. LSM6DS3=0x69)
 */
void host_sensor_models_load_config(const char *path);

/* Handle SPI transfer routed to `device_name`. tx/len as received by
 * host_spi_transfer. Returns 0 on success. */
int host_sensor_handle_spi_transfer(const char *device_name, const uint8_t *tx, uint8_t *rx, int len);

#endif
