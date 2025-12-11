#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t type;
    int16_t arg1;
    int16_t arg2;
} command_t;

int command_processor_init(void);
int command_processor_parse(const uint8_t *data, uint16_t len, command_t *cmd);
bool command_processor_is_timeout(void);

#endif // COMMAND_PROCESSOR_H
