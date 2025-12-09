#ifndef LVDS_H
#define LVDS_H
// LVDS inter-IC comms API
void lvds_init(void);
int lvds_send(const void *data, int len);
int lvds_recv(void *data, int maxlen);
#endif
