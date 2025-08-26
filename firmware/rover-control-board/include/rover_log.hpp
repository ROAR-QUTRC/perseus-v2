#pragma once

// C lib
#include <stdint.h>

// user code
#include "rover_vt100.hpp"
#include "sdkconfig.h"

#define LOG_DEBUG_COLOR VT100_COLOR_GREEN
#define LOG_INFO_COLOR  VT100_COLOR_WHITE
#define LOG_WARN_COLOR  VT100_COLOR_YELLOW
#define LOG_ERROR_COLOR VT100_COLOR_RED

void logPrint(const char* prefix, const char* tag, const uint8_t color, const char* fmt, ...);

#ifdef CONFIG_LOG_DEBUG
#define CONFIG_LOG_INFO
#define DEBUG_TAGGED(tag, ...) logPrint(CONFIG_LOG_PREFIX_DEBUG, tag, LOG_DEBUG_COLOR, __VA_ARGS__)
#else
#define DEBUG_TAGGED(...)
#endif
#ifdef CONFIG_LOG_INFO
#define CONFIG_LOG_WARN
#define INFO_TAGGED(tag, ...) logPrint(CONFIG_LOG_PREFIX_INFO, tag, LOG_INFO_COLOR, __VA_ARGS__)
#else
#define INFO_TAGGED(...)
#endif
#ifdef CONFIG_LOG_WARN
#define CONFIG_LOG_ERROR
#define WARN_TAGGED(tag, ...) logPrint(CONFIG_LOG_PREFIX_WARN, tag, LOG_WARN_COLOR, __VA_ARGS__)
#else
#define WARN_TAGGED(...)
#endif
#ifdef CONFIG_LOG_ERROR
#define ERROR_TAGGED(tag, ...) logPrint(CONFIG_LOG_PREFIX_ERROR, tag, LOG_ERROR_COLOR, __VA_ARGS__)
#else
#define ERROR_TAGGED(...)
#endif

#ifdef CONFIG_LOG_APP
#define DEBUG(...) DEBUG_TAGGED(CONFIG_LOG_TAG_APP, __VA_ARGS__)
#define INFO(...)  INFO_TAGGED(CONFIG_LOG_TAG_APP, __VA_ARGS__)
#define WARN(...)  WARN_TAGGED(CONFIG_LOG_TAG_APP, __VA_ARGS__)
#define ERROR(...) ERROR_TAGGED(CONFIG_LOG_TAG_APP, __VA_ARGS__)
#else
#define DEBUG(...)
#define INFO(...)
#define WARN(...)
#define ERROR(...)
#endif

#ifdef CONFIG_LOG_CORE
#define CORE_DEBUG(...) DEBUG_TAGGED(CONFIG_LOG_TAG_CORE, __VA_ARGS__)
#define CORE_INFO(...)  INFO_TAGGED(CONFIG_LOG_TAG_CORE, __VA_ARGS__)
#define CORE_WARN(...)  WARN_TAGGED(CONFIG_LOG_TAG_CORE, __VA_ARGS__)
#define CORE_ERROR(...) ERROR_TAGGED(CONFIG_LOG_TAG_CORE, __VA_ARGS__)
#else
#define CORE_DEBUG(...)
#define CORE_INFO(...)
#define CORE_WARN(...)
#define CORE_ERROR(...)
#endif

#ifdef CONFIG_LOG_WIFI
#define WIFI_DEBUG(...) DEBUG_TAGGED(CONFIG_LOG_TAG_WIFI, __VA_ARGS__)
#define WIFI_INFO(...)  INFO_TAGGED(CONFIG_LOG_TAG_WIFI, __VA_ARGS__)
#define WIFI_WARN(...)  WARN_TAGGED(CONFIG_LOG_TAG_WIFI, __VA_ARGS__)
#define WIFI_ERROR(...) ERROR_TAGGED(CONFIG_LOG_TAG_WIFI, __VA_ARGS__)
#else
#define WIFI_DEBUG(...)
#define WIFI_INFO(...)
#define WIFI_WARN(...)
#define WIFI_ERROR(...)
#endif

#ifdef CONFIG_LOG_SERVER
#define SERVER_DEBUG(...) DEBUG_TAGGED(CONFIG_LOG_TAG_SERVER, __VA_ARGS__)
#define SERVER_INFO(...)  INFO_TAGGED(CONFIG_LOG_TAG_SERVER, __VA_ARGS__)
#define SERVER_WARN(...)  WARN_TAGGED(CONFIG_LOG_TAG_SERVER, __VA_ARGS__)
#define SERVER_ERROR(...) ERROR_TAGGED(CONFIG_LOG_TAG_SERVER, __VA_ARGS__)
#else
#define SERVER_DEBUG(...)
#define SERVER_INFO(...)
#define SERVER_WARN(...)
#define SERVER_ERROR(...)
#endif

#ifdef CONFIG_LOG_BT
#define BT_DEBUG(...) DEBUG_TAGGED(CONFIG_LOG_TAG_BT, __VA_ARGS__)
#define BT_INFO(...)  INFO_TAGGED(CONFIG_LOG_TAG_BT, __VA_ARGS__)
#define BT_WARN(...)  WARN_TAGGED(CONFIG_LOG_TAG_BT, __VA_ARGS__)
#define BT_ERROR(...) ERROR_TAGGED(CONFIG_LOG_TAG_BT, __VA_ARGS__)
#else
#define BT_DEBUG(...)
#define BT_INFO(...)
#define BT_WARN(...)
#define BT_ERROR(...)
#endif

#ifdef CONFIG_LOG_IO
#define IO_DEBUG(...) DEBUG_TAGGED(CONFIG_LOG_TAG_IO, __VA_ARGS__)
#define IO_INFO(...)  INFO_TAGGED(CONFIG_LOG_TAG_IO, __VA_ARGS__)
#define IO_WARN(...)  WARN_TAGGED(CONFIG_LOG_TAG_IO, __VA_ARGS__)
#define IO_ERROR(...) ERROR_TAGGED(CONFIG_LOG_TAG_IO, __VA_ARGS__)
#else
#define IO_DEBUG(...)
#define IO_INFO(...)
#define IO_WARN(...)
#define IO_ERROR(...)
#endif

#ifdef CONFIG_LOG_CANLIB
#define CANLIB_DEBUG(...) DEBUG_TAGGED(CONFIG_LOG_TAG_CANLIB, __VA_ARGS__)
#define CANLIB_INFO(...)  INFO_TAGGED(CONFIG_LOG_TAG_CANLIB, __VA_ARGS__)
#define CANLIB_WARN(...)  WARN_TAGGED(CONFIG_LOG_TAG_CANLIB, __VA_ARGS__)
#define CANLIB_ERROR(...) ERROR_TAGGED(CONFIG_LOG_TAG_CANLIB, __VA_ARGS__)
#else
#define CANLIB_DEBUG(...)
#define CANLIB_INFO(...)
#define CANLIB_WARN(...)
#define CANLIB_ERROR(...)
#endif