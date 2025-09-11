#ifndef INIT_H
#define INIT_H

void drivers_init(void);
void middleware_init(void);
void app_init(void);
void app_main_loop(void);

// Flight controller functions
int flight_controller_stabilize(const void *attitude, const void *command, void *outputs);
int flight_controller_altitude_hold(const void *attitude, const void *command, void *outputs);
int flight_controller_autonomous(const void *attitude, const void *command, void *outputs);

#endif
