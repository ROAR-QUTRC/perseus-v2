#pragma once

#include <cstdint>

/// Shared system status populated by main loop, read by USB REPL streaming
struct SystemStatus
{
    bool radio_connected;
    int8_t rssi_dbm;
    uint16_t current_ma;
    int16_t servo_speed;  // -1000..+1000
    uint8_t heater_duty;  // 0..255
    bool servo_ok;
    bool watchdog_tripped;
    uint32_t uptime_ms;
    uint32_t last_cmd_age_ms;    // ms since last valid command (0xFFFFFFFF = never)
    uint32_t rx_cmd_count;       // total valid commands received
    uint32_t tx_telem_count;     // total telemetry frames sent
    uint32_t rx_invalid_count;   // corrupted/invalid frames
    uint8_t last_cmd_type;       // last command type byte
    uint8_t last_bad_frame[16];  // last invalid RX frame (debug)
    uint8_t last_bad_len;        // length of last bad frame
};
