#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "as7343_driver/as7343_registers.hpp"
#include "as7343_driver/i2c_device.hpp"
#include "as7343_driver/spectral_data.hpp"

namespace as7343_driver
{

    struct As7343Config
    {
        uint8_t atime{29};    // Integration time multiplier (0-255)
        uint16_t astep{599};  // Integration step period (0-65535)
        Gain gain{Gain::GAIN_256X};
        uint8_t smux_mode{18};  // 6, 12, or 18 channel mode
        bool led_enabled{false};
        uint8_t led_current_ma{4};
        bool flicker_detection_enabled{true};
    };

    class As7343Device
    {
    public:
        explicit As7343Device(std::unique_ptr<I2cDevice> i2c_device);

        bool initialize(const As7343Config& config);
        bool verify_device_id();

        bool set_gain(Gain gain);
        bool set_integration_time(uint8_t atime, uint16_t astep);
        bool set_smux_mode(uint8_t channels);
        bool set_led(bool enabled, uint8_t current_ma);
        bool enable_flicker_detection(bool enabled);

        std::optional<SpectralReading> read_spectral_data();
        std::optional<FlickerReading> read_flicker_status();

        double get_integration_time_ms() const;
        uint16_t get_gain_multiplier() const;

        bool is_initialized() const { return _initialized; }

    private:
        bool _set_bank(uint8_t bank);
        bool _enable_spectral_measurement(bool enable);
        bool _wait_for_data_ready(int timeout_ms);
        bool _software_reset();

        std::unique_ptr<I2cDevice> _i2c;
        As7343Config _config;
        uint8_t _current_bank{0};
        bool _initialized{false};
    };

}  // namespace as7343_driver
