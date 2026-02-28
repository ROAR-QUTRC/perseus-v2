#include "transceiver.hpp"

#include <cstdio>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

namespace transceiver
{

    // Register addresses
    constexpr uint8_t REG_WHOAMI = 0x01;
    constexpr uint8_t REG_NODE_ID_W = 0x95;
    constexpr uint8_t REG_NETWORK_ID_W = 0x96;
    constexpr uint8_t REG_DEST_NODE_W = 0x97;
    constexpr uint8_t REG_TX_POWER_W = 0x98;
    constexpr uint8_t REG_PAYLOAD_LEN_R = 0x21;
    constexpr uint8_t REG_PAYLOAD_LEN_W = 0xA1;
    constexpr uint8_t REG_PAYLOAD_R = 0x22;
    constexpr uint8_t REG_PAYLOAD_W = 0xA2;
    constexpr uint8_t REG_PAYLOAD_NEW = 0x23;
    constexpr uint8_t REG_PAYLOAD_GO = 0xA4;
    constexpr uint8_t REG_TRANSCEIVER_READY = 0x25;
    constexpr uint8_t REG_RSSI = 0x24;

    constexpr uint16_t EXPECTED_WHOAMI = 495;
    constexpr uint8_t I2C_CHUNK_SIZE = 31;

    Transceiver::Transceiver(i2c_inst_t* i2c, uint8_t address)
        : _i2c(i2c),
          _address(address),
          _connected(false)
    {
    }

    bool Transceiver::init(uint8_t sda_pin, uint8_t scl_pin, uint32_t baudrate)
    {
        i2c_init(_i2c, baudrate);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);

        sleep_ms(100);  // Allow transceiver to boot

        uint16_t whoami = read_reg_u16(REG_WHOAMI);
        if (whoami != EXPECTED_WHOAMI)
        {
            printf("Transceiver WHOAMI mismatch: expected %u, got %u\n",
                   EXPECTED_WHOAMI, whoami);
            return false;
        }

        _connected = true;
        printf("Transceiver connected at 0x%02X\n", _address);
        return true;
    }

    void Transceiver::configure(uint8_t node_id, uint8_t network_id,
                                uint8_t dest_node_id, uint8_t tx_power_dbm)
    {
        write_reg_u8(REG_NODE_ID_W, node_id);
        sleep_ms(5);
        write_reg_u8(REG_NETWORK_ID_W, network_id);
        sleep_ms(5);
        write_reg_u8(REG_DEST_NODE_W, dest_node_id);
        sleep_ms(5);
        write_reg_u8(REG_TX_POWER_W, tx_power_dbm);
        sleep_ms(5);
        printf("Configured: node=%u network=%u dest=%u power=%udBm\n", node_id,
               network_id, dest_node_id, tx_power_dbm);
    }

    bool Transceiver::send(const uint8_t* data, uint8_t length)
    {
        if (length > 61)
        {
            return false;
        }

        if (!wait_ready(100))
        {
            return false;
        }

        // Write payload length
        write_reg_u8(REG_PAYLOAD_LEN_W, length);

        // Write payload in chunks
        for (uint8_t offset = 0; offset < length; offset += I2C_CHUNK_SIZE)
        {
            uint8_t chunk_size =
                (length - offset > I2C_CHUNK_SIZE) ? I2C_CHUNK_SIZE : (length - offset);
            uint8_t buf[I2C_CHUNK_SIZE + 1];
            buf[0] = REG_PAYLOAD_W;
            for (uint8_t i = 0; i < chunk_size; i++)
            {
                buf[i + 1] = data[offset + i];
            }
            int ret =
                i2c_write_blocking(_i2c, _address, buf, chunk_size + 1, false);
            if (ret < 0)
            {
                _connected = false;
                return false;
            }
        }

        // Trigger send
        write_reg_u8(REG_PAYLOAD_GO, 1);
        return true;
    }

    uint8_t Transceiver::receive(uint8_t* buf, uint8_t max_length)
    {
        uint8_t new_flag = read_reg_u8(REG_PAYLOAD_NEW);
        if (!new_flag)
        {
            return 0;
        }

        uint8_t length = read_reg_u8(REG_PAYLOAD_LEN_R);
        if (length == 0 || length > 61 || length > max_length)
        {
            return 0;
        }

        // Read payload in chunks
        uint8_t read_total = 0;
        while (read_total < length)
        {
            uint8_t chunk_size = (length - read_total > I2C_CHUNK_SIZE)
                                     ? I2C_CHUNK_SIZE
                                     : (length - read_total);
            uint8_t reg = REG_PAYLOAD_R;
            int ret = i2c_write_blocking(_i2c, _address, &reg, 1, true);
            if (ret < 0)
            {
                _connected = false;
                return 0;
            }
            ret = i2c_read_blocking(_i2c, _address, buf + read_total, chunk_size,
                                    false);
            if (ret < 0)
            {
                _connected = false;
                return 0;
            }
            read_total += chunk_size;
        }

        return length;
    }

    int8_t Transceiver::read_rssi()
    {
        uint8_t raw = read_reg_u8(REG_RSSI);
        return static_cast<int8_t>(raw);
    }

    bool Transceiver::write_reg_u8(uint8_t reg, uint8_t value)
    {
        uint8_t buf[2] = {reg, value};
        int ret = i2c_write_blocking(_i2c, _address, buf, 2, false);
        return ret >= 0;
    }

    uint8_t Transceiver::read_reg_u8(uint8_t reg)
    {
        uint8_t value = 0;
        i2c_write_blocking(_i2c, _address, &reg, 1, true);
        i2c_read_blocking(_i2c, _address, &value, 1, false);
        return value;
    }

    uint16_t Transceiver::read_reg_u16(uint8_t reg)
    {
        uint8_t buf[2] = {0, 0};
        i2c_write_blocking(_i2c, _address, &reg, 1, true);
        i2c_read_blocking(_i2c, _address, buf, 2, false);
        return (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    }

    bool Transceiver::wait_ready(uint32_t timeout_ms)
    {
        absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
        while (!time_reached(deadline))
        {
            if (read_reg_u8(REG_TRANSCEIVER_READY))
            {
                return true;
            }
            sleep_ms(2);
        }
        return false;
    }

}  // namespace transceiver
