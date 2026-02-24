#include "as7343_driver/as7343_node.hpp"

#include <stdexcept>
#include <thread>

namespace as7343_driver
{

    As7343Node::As7343Node(const rclcpp::NodeOptions& options)
        : Node("as7343_node", options)
    {
        _initialize_parameters();
        _initialize_publishers();

        if (!_initialize_device())
        {
            if (_required)
            {
                throw std::runtime_error("Failed to initialize required AS7343 spectral sensor");
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Failed to initialize optional AS7343 sensor, continuing without it");
                return;
            }
        }

        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / _publish_rate_hz));
        _timer = create_wall_timer(timer_period, std::bind(&As7343Node::_timer_callback, this));

        RCLCPP_INFO(get_logger(), "AS7343 spectral sensor node initialized");
        RCLCPP_INFO(get_logger(), "  I2C bus: %s, address: 0x%02X", _i2c_bus_path.c_str(), _device_address);
        RCLCPP_INFO(get_logger(), "  Gain: %ux, Integration time: %.2f ms",
                    _device->get_gain_multiplier(), _device->get_integration_time_ms());
        RCLCPP_INFO(get_logger(), "  SMUX mode: %d channels, Publish rate: %.1f Hz",
                    _smux_mode, _publish_rate_hz);
    }

    As7343Node::~As7343Node()
    {
        if (_timer)
        {
            _timer->cancel();
        }
    }

    void As7343Node::_initialize_parameters()
    {
        // I2C configuration
        declare_parameter("i2c_bus", "/dev/i2c-1");
        declare_parameter("device_address", 0x39);
        declare_parameter("publish_rate_hz", 5.0);
        declare_parameter("frame_id", "as7343_link");
        declare_parameter("required", false);
        declare_parameter("retry_count", 3);

        // Sensor configuration
        declare_parameter("gain", 256);
        declare_parameter("atime", 29);
        declare_parameter("astep", 599);
        declare_parameter("smux_mode", 18);
        declare_parameter("led_enabled", false);
        declare_parameter("led_current_ma", 4);
        declare_parameter("flicker_detection_enabled", true);

        // Get parameter values
        _i2c_bus_path = get_parameter("i2c_bus").as_string();

        int device_address_int = get_parameter("device_address").as_int();
        if (device_address_int < 0 || device_address_int > 127)
        {
            throw std::invalid_argument("I2C device address must be 0-127, got: " +
                                        std::to_string(device_address_int));
        }
        _device_address = static_cast<uint8_t>(device_address_int);

        _publish_rate_hz = get_parameter("publish_rate_hz").as_double();
        _frame_id = get_parameter("frame_id").as_string();
        _required = get_parameter("required").as_bool();
        _retry_count = get_parameter("retry_count").as_int();

        _gain = static_cast<uint16_t>(get_parameter("gain").as_int());
        _atime = static_cast<uint8_t>(get_parameter("atime").as_int());
        _astep = static_cast<uint16_t>(get_parameter("astep").as_int());
        _smux_mode = static_cast<uint8_t>(get_parameter("smux_mode").as_int());
        _led_enabled = get_parameter("led_enabled").as_bool();
        _led_current_ma = static_cast<uint8_t>(get_parameter("led_current_ma").as_int());
        _flicker_detection_enabled = get_parameter("flicker_detection_enabled").as_bool();

        // Validate parameters
        if (_publish_rate_hz <= 0.0)
        {
            throw std::invalid_argument("Publish rate must be positive");
        }
        if (_retry_count < 0)
        {
            throw std::invalid_argument("Retry count must be non-negative");
        }
        if (_smux_mode != 6 && _smux_mode != 12 && _smux_mode != 18)
        {
            throw std::invalid_argument("SMUX mode must be 6, 12, or 18");
        }
    }

    void As7343Node::_initialize_publishers()
    {
        _spectral_pub = create_publisher<perseus_interfaces::msg::SpectralData>("~/spectral_data", 10);
        _flicker_pub = create_publisher<perseus_interfaces::msg::FlickerStatus>("~/flicker_status", 10);
        _integration_time_pub = create_publisher<std_msgs::msg::Float64>("~/integration_time_ms", 10);
    }

    bool As7343Node::_initialize_device()
    {
        int retry_count = 0;

        while (retry_count < _retry_count)
        {
            try
            {
                auto i2c = std::make_unique<I2cDevice>(_i2c_bus_path, _device_address);
                _device = std::make_unique<As7343Device>(std::move(i2c));

                As7343Config config;
                config.atime = _atime;
                config.astep = _astep;
                config.gain = gain_from_multiplier(_gain);
                config.smux_mode = _smux_mode;
                config.led_enabled = _led_enabled;
                config.led_current_ma = _led_current_ma;
                config.flicker_detection_enabled = _flicker_detection_enabled;

                if (_device->initialize(config))
                {
                    _device_initialized = true;
                    RCLCPP_INFO(get_logger(), "Successfully initialized AS7343 at %s:0x%02X",
                                _i2c_bus_path.c_str(), _device_address);
                    return true;
                }

                throw std::runtime_error("Device initialization returned false");
            }
            catch (const std::exception& e)
            {
                retry_count++;
                if (retry_count < _retry_count)
                {
                    RCLCPP_WARN(get_logger(), "AS7343 initialization failed (attempt %d/%d): %s",
                                retry_count, _retry_count, e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "AS7343 initialization failed after %d attempts: %s",
                                 _retry_count, e.what());
                }
            }
        }

        return false;
    }

    void As7343Node::_timer_callback()
    {
        if (!_device_initialized || !_device)
        {
            return;
        }

        _publish_spectral_data();

        if (_flicker_detection_enabled)
        {
            _publish_flicker_status();
        }
    }

    void As7343Node::_publish_spectral_data()
    {
        auto reading = _device->read_spectral_data();
        if (!reading.has_value())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Failed to read spectral data from AS7343");
            return;
        }

        const auto& data = reading.value();

        perseus_interfaces::msg::SpectralData msg;
        msg.header.stamp = now();
        msg.header.frame_id = _frame_id;

        // Spectral channels
        msg.f1_405nm = data.f1_405nm;
        msg.f2_425nm = data.f2_425nm;
        msg.fz_450nm = data.fz_450nm;
        msg.f3_475nm = data.f3_475nm;
        msg.f4_515nm = data.f4_515nm;
        msg.f5_550nm = data.f5_550nm;
        msg.fy_555nm = data.fy_555nm;
        msg.fxl_600nm = data.fxl_600nm;
        msg.f6_640nm = data.f6_640nm;
        msg.f7_690nm = data.f7_690nm;
        msg.f8_745nm = data.f8_745nm;
        msg.nir_855nm = data.nir_855nm;
        msg.vis_clear = data.vis_clear;
        msg.fd_flicker = data.fd_flicker;

        // Status
        msg.analog_saturation = data.analog_saturation;
        msg.digital_saturation = data.digital_saturation;
        msg.data_valid = data.data_valid;

        // Current configuration
        msg.integration_time_ms = _device->get_integration_time_ms();
        msg.gain = _device->get_gain_multiplier();

        _spectral_pub->publish(msg);

        // Publish integration time as a separate topic for easy monitoring
        std_msgs::msg::Float64 time_msg;
        time_msg.data = _device->get_integration_time_ms();
        _integration_time_pub->publish(time_msg);

        _sequence_number++;

        if (data.analog_saturation || data.digital_saturation)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "AS7343 saturation detected (analog: %s, digital: %s). Consider reducing gain or integration time.",
                                 data.analog_saturation ? "yes" : "no",
                                 data.digital_saturation ? "yes" : "no");
        }
    }

    void As7343Node::_publish_flicker_status()
    {
        auto reading = _device->read_flicker_status();
        if (!reading.has_value())
        {
            return;
        }

        const auto& data = reading.value();

        perseus_interfaces::msg::FlickerStatus msg;
        msg.header.stamp = now();
        msg.header.frame_id = _frame_id;

        msg.detected_frequency_hz = data.detected_frequency_hz;
        msg.hz_100_valid = data.hz_100_valid;
        msg.hz_120_valid = data.hz_120_valid;
        msg.hz_100_detected = data.hz_100_detected;
        msg.hz_120_detected = data.hz_120_detected;
        msg.fd_saturation = data.fd_saturation;
        msg.fd_valid = data.fd_valid;

        _flicker_pub->publish(msg);
    }

}  // namespace as7343_driver
