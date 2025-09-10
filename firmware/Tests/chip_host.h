/* Minimal chip.h shim for host builds to provide LPC_GPIO_PORT, clock and small helpers
 * This header is included when building firmware for host tests. It defines minimal
 * macros and functions referenced by drivers. */

#ifndef CHIP_HOST_H
#define CHIP_HOST_H

#include <stdint.h>

/* Minimal GPIO port simulation */
typedef struct {
    uint32_t SET[4];
    uint32_t CLR[4];
} LPC_GPIO_PORT_Type;

extern LPC_GPIO_PORT_Type *LPC_GPIO_PORT;

/* Busy wait */
void Chip_Clock_System_BusyWait_ms(uint32_t ms);

/* I2C stubs used by drivers when compiled for host */
int Chip_I2C_MasterSend(void *i2c, uint8_t addr, const uint8_t *data, int len);
int Chip_I2C_MasterRead(void *i2c, uint8_t addr, uint8_t *data, int len);

#endif
