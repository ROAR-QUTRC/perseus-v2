#include "as7343_driver/as7343_device.hpp"

#include <chrono>
#include <thread>

namespace as7343_driver
{

    As7343Device::As7343Device(std::unique_ptr<I2cDevice> i2c_device)
        : _i2c(std::move(i2c_device))
    {
    }

    bool As7343Device::initialize(const As7343Config& config)
    {
        _config = config;
        _initialized = false;

        if (!_i2c || !_i2c->is_connected())
        {
            return false;
        }

        if (!verify_device_id())
        {
            return false;
        }

        if (!_software_reset())
        {
            return false;
        }

        // Power on
        if (!_i2c->write_register(REG_ENABLE, ENABLE_PON))
        {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Set gain
        if (!set_gain(_config.gain))
        {
            return false;
        }

        // Set integration time
        if (!set_integration_time(_config.atime, _config.astep))
        {
            return false;
        }

        // Set SMUX mode for auto measurement cycling
        if (!set_smux_mode(_config.smux_mode))
        {
            return false;
        }

        // Configure LED if requested
        if (_config.led_enabled)
        {
            if (!set_led(true, _config.led_current_ma))
            {
                return false;
            }
        }

        // Enable flicker detection if requested
        if (_config.flicker_detection_enabled)
        {
            if (!enable_flicker_detection(true))
            {
                return false;
            }
        }

        _initialized = true;
        return true;
    }

    bool As7343Device::verify_device_id()
    {
        // Switch to bank 1 to read ID register
        if (!_set_bank(1))
        {
            return false;
        }

        auto id = _i2c->read_register(REG_ID);
        if (!id.has_value())
        {
            return false;
        }

        // Switch back to bank 0
        _set_bank(0);

        return id.value() == AS7343_DEVICE_ID;
    }

    bool As7343Device::_set_bank(uint8_t bank)
    {
        if (_current_bank == bank && _initialized)
        {
            return true;
        }

        auto cfg0 = _i2c->read_register(REG_CFG0);
        if (!cfg0.has_value())
        {
            return false;
        }

        uint8_t new_cfg0;
        if (bank == 1)
        {
            new_cfg0 = cfg0.value() | CFG0_REG_BANK;
        }
        else
        {
            new_cfg0 = cfg0.value() & ~CFG0_REG_BANK;
        }

        if (!_i2c->write_register(REG_CFG0, new_cfg0))
        {
            return false;
        }

        _current_bank = bank;
        return true;
    }

    bool As7343Device::_software_reset()
    {
        // Ensure bank 0
        if (!_set_bank(0))
        {
            return false;
        }

        if (!_i2c->write_register(REG_CONTROL, CONTROL_SW_RESET))
        {
            return false;
        }

        // Wait for reset to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Poll for device presence after reset
        for (int i = 0; i < 10; ++i)
        {
            if (_i2c->is_connected())
            {
                auto enable_val = _i2c->read_register(REG_ENABLE);
                if (enable_val.has_value())
                {
                    _current_bank = 0;
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        return false;
    }

    bool As7343Device::set_gain(Gain gain)
    {
        if (!_set_bank(0))
        {
            return false;
        }

        if (!_i2c->write_register(REG_CFG1, static_cast<uint8_t>(gain)))
        {
            return false;
        }

        _config.gain = gain;
        return true;
    }

    bool As7343Device::set_integration_time(uint8_t atime, uint16_t astep)
    {
        if (!_set_bank(0))
        {
            return false;
        }

        // Write ATIME
        if (!_i2c->write_register(REG_ATIME, atime))
        {
            return false;
        }

        // Write ASTEP (16-bit, little-endian)
        uint8_t astep_bytes[2] = {
            static_cast<uint8_t>(astep & 0xFF),
            static_cast<uint8_t>((astep >> 8) & 0xFF)};

        if (!_i2c->write_registers(REG_ASTEP_L, astep_bytes, 2))
        {
            return false;
        }

        _config.atime = atime;
        _config.astep = astep;
        return true;
    }

    bool As7343Device::set_smux_mode(uint8_t channels)
    {
        if (!_set_bank(0))
        {
            return false;
        }

        auto cfg20 = _i2c->read_register(REG_CFG20);
        if (!cfg20.has_value())
        {
            return false;
        }

        uint8_t new_cfg20 = cfg20.value() & 0x9F;  // Clear bits 6:5

        switch (channels)
        {
        case 6:
            new_cfg20 |= CFG20_SMUX_6CH;
            break;
        case 12:
            new_cfg20 |= CFG20_SMUX_12CH;
            break;
        case 18:
        default:
            new_cfg20 |= CFG20_SMUX_18CH;
            break;
        }

        if (!_i2c->write_register(REG_CFG20, new_cfg20))
        {
            return false;
        }

        _config.smux_mode = channels;
        return true;
    }

    bool As7343Device::set_led(bool enabled, uint8_t current_ma)
    {
        if (!_set_bank(0))
        {
            return false;
        }

        uint8_t led_reg = 0;
        if (enabled)
        {
            // LED current: current_mA = 4 + (register_value * 2)
            // register_value = (current_mA - 4) / 2
            uint8_t current_val = 0;
            if (current_ma >= 4)
            {
                current_val = (current_ma - 4) / 2;
                if (current_val > 127)
                {
                    current_val = 127;
                }
            }
            led_reg = 0x80 | current_val;  // Bit 7 = LED_ACT
        }

        if (!_i2c->write_register(REG_LED, led_reg))
        {
            return false;
        }

        _config.led_enabled = enabled;
        _config.led_current_ma = current_ma;
        return true;
    }

    bool As7343Device::enable_flicker_detection(bool enabled)
    {
        if (!_set_bank(0))
        {
            return false;
        }

        auto enable_reg = _i2c->read_register(REG_ENABLE);
        if (!enable_reg.has_value())
        {
            return false;
        }

        uint8_t new_enable;
        if (enabled)
        {
            new_enable = enable_reg.value() | ENABLE_FDEN;
        }
        else
        {
            new_enable = enable_reg.value() & ~ENABLE_FDEN;
        }

        if (!_i2c->write_register(REG_ENABLE, new_enable))
        {
            return false;
        }

        _config.flicker_detection_enabled = enabled;
        return true;
    }

    bool As7343Device::_enable_spectral_measurement(bool enable)
    {
        auto enable_reg = _i2c->read_register(REG_ENABLE);
        if (!enable_reg.has_value())
        {
            return false;
        }

        uint8_t new_enable;
        if (enable)
        {
            new_enable = enable_reg.value() | ENABLE_SP_EN;
        }
        else
        {
            new_enable = enable_reg.value() & ~ENABLE_SP_EN;
        }

        return _i2c->write_register(REG_ENABLE, new_enable);
    }

    bool As7343Device::_wait_for_data_ready(int timeout_ms)
    {
        auto start = std::chrono::steady_clock::now();

        while (true)
        {
            auto status2 = _i2c->read_register(REG_STATUS2);
            if (status2.has_value() && (status2.value() & STATUS2_AVALID))
            {
                return true;
            }

            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            if (elapsed.count() >= timeout_ms)
            {
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    std::optional<SpectralReading> As7343Device::read_spectral_data()
    {
        if (!_initialized || !_i2c || !_i2c->is_connected())
        {
            return std::nullopt;
        }

        if (!_set_bank(0))
        {
            return std::nullopt;
        }

        // Enable spectral measurement
        if (!_enable_spectral_measurement(true))
        {
            return std::nullopt;
        }

        // Wait for data ready with timeout based on integration time
        // In 18-channel mode, takes 3x integration time
        int timeout = static_cast<int>(get_integration_time_ms() * 3) + 500;
        if (!_wait_for_data_ready(timeout))
        {
            _enable_spectral_measurement(false);
            return std::nullopt;
        }

        // Read ASTATUS to latch data
        auto astatus = _i2c->read_register(REG_ASTATUS);
        if (!astatus.has_value())
        {
            _enable_spectral_measurement(false);
            return std::nullopt;
        }

        // Read STATUS2 for saturation flags
        auto status2 = _i2c->read_register(REG_STATUS2);

        // Burst read all 36 bytes of channel data
        uint8_t raw_data[AS7343_DATA_BYTES];
        if (!_i2c->read_registers(REG_DATA0_L, raw_data, AS7343_DATA_BYTES))
        {
            _enable_spectral_measurement(false);
            return std::nullopt;
        }

        // Disable spectral measurement
        _enable_spectral_measurement(false);

        // Clear STATUS by reading and writing back
        auto status = _i2c->read_register(REG_STATUS);
        if (status.has_value())
        {
            _i2c->write_register(REG_STATUS, status.value());
        }

        // Parse 18 little-endian uint16 values
        uint16_t channels[AS7343_NUM_DATA_REGISTERS];
        for (size_t i = 0; i < AS7343_NUM_DATA_REGISTERS; ++i)
        {
            channels[i] = static_cast<uint16_t>(raw_data[i * 2]) |
                          (static_cast<uint16_t>(raw_data[i * 2 + 1]) << 8);
        }

        SpectralReading reading;
        reading.timestamp = std::chrono::steady_clock::now();

        // Map channels to spectral bands
        reading.fz_450nm = channels[IDX_FZ];
        reading.fy_555nm = channels[IDX_FY];
        reading.fxl_600nm = channels[IDX_FXL];
        reading.nir_855nm = channels[IDX_NIR];
        reading.f2_425nm = channels[IDX_F2];
        reading.f3_475nm = channels[IDX_F3];
        reading.f4_515nm = channels[IDX_F4];
        reading.f6_640nm = channels[IDX_F6];
        reading.f1_405nm = channels[IDX_F1];
        reading.f7_690nm = channels[IDX_F7];
        reading.f8_745nm = channels[IDX_F8];
        reading.f5_550nm = channels[IDX_F5];

        // Average clear and flicker channels across 3 cycles
        reading.vis_clear = static_cast<uint16_t>(
            (static_cast<uint32_t>(channels[IDX_VIS_1]) +
             channels[IDX_VIS_2] + channels[IDX_VIS_3]) /
            3);
        reading.fd_flicker = static_cast<uint16_t>(
            (static_cast<uint32_t>(channels[IDX_FD_1]) +
             channels[IDX_FD_2] + channels[IDX_FD_3]) /
            3);

        // Status flags
        if (status2.has_value())
        {
            reading.analog_saturation = (status2.value() & STATUS2_ASAT_ANALOG) != 0;
            reading.digital_saturation = (status2.value() & STATUS2_ASAT_DIGITAL) != 0;
            reading.data_valid = (status2.value() & STATUS2_AVALID) != 0;
        }

        return reading;
    }

    std::optional<FlickerReading> As7343Device::read_flicker_status()
    {
        if (!_initialized || !_i2c || !_i2c->is_connected())
        {
            return std::nullopt;
        }

        if (!_set_bank(0))
        {
            return std::nullopt;
        }

        auto fd_status = _i2c->read_register(REG_FD_STATUS);
        if (!fd_status.has_value())
        {
            return std::nullopt;
        }

        FlickerReading reading;
        reading.timestamp = std::chrono::steady_clock::now();

        uint8_t status = fd_status.value();
        reading.hz_100_detected = (status & FD_STATUS_100HZ_DET) != 0;
        reading.hz_120_detected = (status & FD_STATUS_120HZ_DET) != 0;
        reading.hz_100_valid = (status & FD_STATUS_100HZ_VALID) != 0;
        reading.hz_120_valid = (status & FD_STATUS_120HZ_VALID) != 0;
        reading.fd_valid = (status & FD_STATUS_FD_MEAS_VALID) != 0;
        reading.fd_saturation = (status & FD_STATUS_FD_SATURATION) != 0;

        // Determine detected frequency
        if (reading.hz_100_detected && reading.hz_100_valid)
        {
            reading.detected_frequency_hz = 100;
        }
        else if (reading.hz_120_detected && reading.hz_120_valid)
        {
            reading.detected_frequency_hz = 120;
        }
        else
        {
            reading.detected_frequency_hz = 0;
        }

        return reading;
    }

    double As7343Device::get_integration_time_ms() const
    {
        // t_integration = (ATIME + 1) * (ASTEP + 1) * 2.78 us
        double t_us = static_cast<double>(_config.atime + 1) *
                      static_cast<double>(_config.astep + 1) *
                      ASTEP_PERIOD_US;
        return t_us / 1000.0;
    }

    uint16_t As7343Device::get_gain_multiplier() const
    {
        return gain_to_multiplier(_config.gain);
    }

}  // namespace as7343_driver
