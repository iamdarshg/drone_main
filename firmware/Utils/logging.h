#ifndef LOGGING_H
#define LOGGING_H

#include <stdarg.h>

/* Minimal logging API used by tests and host-side runners. Production builds
 * can replace this with a more advanced logger. */
void log_info(const char *fmt, ...);
void log_warn(const char *fmt, ...);
void log_error(const char *fmt, ...);

#endif
