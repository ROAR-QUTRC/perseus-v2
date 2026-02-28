#pragma once

#include <cstdint>

#include "hardware/i2c.h"

namespace transceiver
{

    /// PiicoDev 915MHz Transceiver (CE08862) I2C driver for RP2040
    class Transceiver
    {
    public:
        Transceiver(i2c_inst_t* i2c, uint8_t address = 0x1A);

        /// Initialize I2C and verify WHOAMI
        bool init(uint8_t sda_pin, uint8_t scl_pin, uint32_t baudrate = 100000);

        /// Configure radio parameters
        void configure(uint8_t node_id, uint8_t network_id, uint8_t dest_node_id,
                       uint8_t tx_power_dbm = 13);

        /// Send payload over radio. Returns true on success.
        bool send(const uint8_t* data, uint8_t length);

        /// Check for received payload. Returns length (0 = no data).
        uint8_t receive(uint8_t* buf, uint8_t max_length);

        /// Read last RSSI value in dBm
        int8_t read_rssi();

        bool is_connected() const { return _connected; }

    private:
        i2c_inst_t* _i2c;
        uint8_t _address;
        bool _connected;
        int16_t _last_rssi;

        bool write_reg_u8(uint8_t reg, uint8_t value);
        uint8_t read_reg_u8(uint8_t reg);
        uint16_t read_reg_u16(uint8_t reg);
        int read_block(uint8_t reg, uint8_t* buf, uint8_t length);
        bool wait_ready(uint32_t timeout_ms = 100);
    };

}  // namespace transceiver
