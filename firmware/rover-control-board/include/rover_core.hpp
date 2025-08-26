#pragma once

// standard libs
#include <cstdint>

// core
#include <esp_err.h>

#define ROVER_CORE_WIFI_STATION "QUTRC Perseus"
#define ROVER_CORE_WIFI_PASSKEY "NASAtookartemis_Perseus-2024-1"

#define ROVER_CORE_RET_ERR_CHECK(_errMsg, _code)             \
    if (esp_err_t _tmpErr = (_code); _tmpErr != ESP_OK)      \
    {                                                        \
        CORE_ERROR(_errMsg " %s", esp_err_to_name(_tmpErr)); \
        return;                                              \
    }
#define ROVER_CORE_ERR_CHECK(_errMsg, _code)                 \
    if (esp_err_t _tmpErr = (_code); _tmpErr != ESP_OK)      \
    {                                                        \
        CORE_ERROR(_errMsg " %s", esp_err_to_name(_tmpErr)); \
    }
#define ROVER_APP_RET_ERR_CHECK(_errMsg, _code)         \
    if (esp_err_t _tmpErr = (_code); _tmpErr != ESP_OK) \
    {                                                   \
        ERROR(_errMsg " %s", esp_err_to_name(_tmpErr)); \
        return;                                         \
    }
#define ROVER_APP_ERR_CHECK(_errMsg, _code)             \
    if (esp_err_t _tmpErr = (_code); _tmpErr != ESP_OK) \
    {                                                   \
        ERROR(_errMsg " %s", esp_err_to_name(_tmpErr)); \
    }

void coreInit();
void coreRestart();

int64_t coreGetUptime();
uint8_t coreGetUsage();

void coreStartWifi(const char* const hostname);
void coreStopWifi();