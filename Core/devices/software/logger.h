#ifndef LOGGER_H
#define LOGGER_H

#include "stdio.h"
#include "stdarg.h"
#include "flash.h"
#include "wireless.h"

typedef enum{
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO = 1,
    LOG_LEVEL_IMPORTANT = 2,
    LOG_LEVEL_WARN = 3,
    LOG_LEVEL_ERROR = 4,
} log_level_t;

int output_log(log_level_t level, const char *format, ...);

#endif