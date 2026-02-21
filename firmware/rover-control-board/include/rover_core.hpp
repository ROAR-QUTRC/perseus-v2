#pragma once

// standard libs
#include <cstdint>

// core
#include <esp_err.h>

#define ROVER_CORE_RET_ERR_CHECK(_error_message, _code)         \
    if (esp_err_t _tmpErr = (_code); _tmpErr != ESP_OK)         \
    {                                                           \
        printf(_error_message " %s", esp_err_to_name(_tmpErr)); \
        return;                                                 \
    }
#define ROVER_CORE_ERR_CHECK(_error_message, _code)             \
    if (esp_err_t _tmpErr = (_code); _tmpErr != ESP_OK)         \
    {                                                           \
        printf(_error_message " %s", esp_err_to_name(_tmpErr)); \
    }
#define ROVER_APP_RET_ERR_CHECK(_error_message, _code)          \
    if (esp_err_t _tmpErr = (_code); _tmpErr != ESP_OK)         \
    {                                                           \
        printf(_error_message " %s", esp_err_to_name(_tmpErr)); \
        return;                                                 \
    }
#define ROVER_APP_ERR_CHECK(_error_message, _code)              \
    if (esp_err_t _tmpErr = (_code); _tmpErr != ESP_OK)         \
    {                                                           \
        printf(_error_message " %s", esp_err_to_name(_tmpErr)); \
    }

void core_init();

int64_t core_get_uptime();
uint8_t core_get_usage();