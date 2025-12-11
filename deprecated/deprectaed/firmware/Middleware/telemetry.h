#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

int telemetry_init(void);
int telemetry_send(const void *pkt, uint16_t len);

#endif // TELEMETRY_H
