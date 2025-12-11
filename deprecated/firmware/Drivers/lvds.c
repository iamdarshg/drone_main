#include "lvds.h"
#include "logging.h"
void lvds_init(void) { log_info("LVDS initialized"); }
int lvds_send(const void *data, int len) { log_info("LVDS send"); return len; }
int lvds_recv(void *data, int maxlen) { log_info("LVDS recv"); return 0; }
