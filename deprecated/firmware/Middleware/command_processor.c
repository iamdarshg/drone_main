#include "command_processor.h"
#include <string.h>

int command_processor_init(void) {
    // minimal init
    return 0;
}

int command_processor_parse(const uint8_t *data, uint16_t len, command_t *cmd) {
    if (!data || len == 0 || !cmd) return -1;
    // Very small parser: first byte type, next two bytes arg1, next two bytes arg2
    cmd->type = data[0];
    if (len >= 3) cmd->arg1 = (int16_t)((data[1] << 8) | data[2]);
    else cmd->arg1 = 0;
    if (len >= 5) cmd->arg2 = (int16_t)((data[3] << 8) | data[4]);
    else cmd->arg2 = 0;
    return 0;
}

bool command_processor_is_timeout(void) {
    return false;
}
