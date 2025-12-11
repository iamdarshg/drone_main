/* Simple logging implementation used by test runners and host tools.
 * This file provides a minimal printf-style logger.
 */

#include "logging.h"
#include <stdio.h>
#include <time.h>

static void log_vprint(FILE *out, const char *level, const char *fmt, va_list ap)
{
    time_t t = time(NULL);
    struct tm tm;
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm);
    fprintf(out, "%s [%s] ", ts, level);
    vfprintf(out, fmt, ap);
    fprintf(out, "\n");
}

void log_info(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_vprint(stdout, "INFO", fmt, ap);
    va_end(ap);
}

void log_warn(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_vprint(stdout, "WARN", fmt, ap);
    va_end(ap);
}

void log_error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_vprint(stderr, "ERROR", fmt, ap);
    va_end(ap);
}
