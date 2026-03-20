// TODO: Clean up _process_ilmenite function
#include "space_resources/ilmenite/ilmenite-station.hpp"

#include <pigpio.h>

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <perseus_interfaces/msg/as7343_data.hpp>
#include <rclcpp/rclcpp/logger.hpp>
#include <rclcpp/rclcpp/logging.hpp>
#include <rclcpp/rclcpp/utilities.hpp>

using namespace std::chrono_literals;

IlmeniteStation::IlmeniteStation(const rclcpp::NodeOptions& options)
    : Node("ilmenite_station", options)
{
    gpioInitialise();
    _spectral_data_client = create_client<perseus_interfaces::srv::GetSpectralData>("/get_spectral_data");
    _led_client = create_client<std_srvs::srv::SetBool>("/set_led_status");
    _read_ilmenite_service = create_service<perseus_interfaces::srv::TakeAs7343Reading>("/read_ilmenite", std::bind(&IlmeniteStation::_process_ilmenite, this, std::placeholders::_1, std::placeholders::_2));
    _ilmenite_set_lid_servo = create_service<std_srvs::srv::SetBool>("/set_lid_servo", std::bind(&IlmeniteStation::_set_lid_servo, this, std::placeholders::_1, std::placeholders::_2));
    _ilmenite_set_tip_servo = create_service<std_srvs::srv::SetBool>("/set_tip_servo", std::bind(&IlmeniteStation::_set_tip_servo, this, std::placeholders::_1, std::placeholders::_2));
    _lid_servo_pin = this->declare_parameter("lid_servo_pin", 0);
    _tip_servo_pin = this->declare_parameter("tip_servo_pin", 1);
    _uv_led_pin = this->declare_parameter("uv_led_pin", 2);

    // Initialize GPIO pins
    gpioSetMode(_lid_servo_pin, PI_OUTPUT);
    gpioServo(_lid_servo_pin, _lid_closed_pulsewidth);
    gpioSetMode(_tip_servo_pin, PI_OUTPUT);
    gpioServo(_tip_servo_pin, _tip_upright_pulsewidth);
    gpioSetMode(_uv_led_pin, PI_OUTPUT);
    gpioWrite(_uv_led_pin, 0);
}

void IlmeniteStation::_set_lid_servo(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    int servo_error_value;
    if (request->data == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Requested state of servo is closed");
        servo_error_value = gpioServo(_lid_servo_pin, _lid_closed_pulsewidth);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Requested state of servo is open");
        servo_error_value = gpioServo(_lid_servo_pin, _lid_open_pulsewidth);
    }

    if (servo_error_value == 0)
    {
        RCLCPP_DEBUG(this->get_logger(), "Set lid servo succeeded");
        response->message = "Success";
        response->success = 1;
    }
    else if (servo_error_value == PI_BAD_USER_GPIO)
    {
        RCLCPP_DEBUG(this->get_logger(), "Set lid servo failed due to bad user GPIO");
        response->message = "Bad user GPIO";
        response->success = 0;
    }
    else if (servo_error_value == PI_BAD_PULSEWIDTH)
    {
        RCLCPP_DEBUG(this->get_logger(), "Set lid servo failed due to bad pulsewidth");
        response->message = "Bad pulsewidth";
        response->success = 0;
    }
}

void IlmeniteStation::_set_tip_servo(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    int servo_error_value;
    if (request->data == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Requested state of tip servo is upright");
        servo_error_value = gpioServo(_tip_servo_pin, _tip_upright_pulsewidth);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Requested state of servo is open");
        servo_error_value = gpioServo(_tip_servo_pin, _tip_upside_down_pulsewidth);
    }

    if (servo_error_value == 0)
    {
        RCLCPP_DEBUG(this->get_logger(), "Set tip servo succeeded");
        response->message = "Success";
        response->success = true;
    }
    else if (servo_error_value == PI_BAD_USER_GPIO)
    {
        RCLCPP_DEBUG(this->get_logger(), "Set tip servo failed due to bad user GPIO");
        response->message = "Bad user GPIO";
        response->success = false;
    }
    else if (servo_error_value == PI_BAD_PULSEWIDTH)
    {
        RCLCPP_DEBUG(this->get_logger(), "Set tip servo failed due to bad pulsewidth");
        response->message = "Bad pulsewidth";
        response->success = false;
    }
}

void IlmeniteStation::_process_ilmenite(const std::shared_ptr<perseus_interfaces::srv::TakeAs7343Reading::Request> /*request*/, std::shared_ptr<perseus_interfaces::srv::TakeAs7343Reading::Response> response)
{
    // Ensure the lid is closed
    gpioServo(_lid_servo_pin, _lid_closed_pulsewidth);
    // Ensure the leds are both off
    gpioWrite(_uv_led_pin, false);
    rclcpp::sleep_for(100ms);
    auto request_led_off = std::make_shared<std_srvs::srv::SetBool::Request>();
    request_led_off->data = false;
    while (!_led_client->wait_for_service(100ms))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted by error while waiting for led service, exiting");
        }
        RCLCPP_INFO(this->get_logger(), "Timed out waiting for led service, waiting again");
    }
    auto led_off_result = _led_client->async_send_request(request_led_off);
    led_off_result.wait();

    while (!_spectral_data_client->wait_for_service(100ms))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted by error while waiting for spectral data service, exiting");
        }
        RCLCPP_INFO(this->get_logger(), "Timed out waiting for spectral data service, waiting again");
    }

    auto led_off_spectral_data_future = _spectral_data_client->async_send_request(std::make_shared<perseus_interfaces::srv::GetSpectralData::Request>());
    led_off_spectral_data_future.wait();
    auto led_off_spectral_data_result = led_off_spectral_data_future.get();
    perseus_interfaces::msg::As7343Data led_off_spectral_data;
    if (!led_off_spectral_data_result->success)
    {
        RCLCPP_ERROR(this->get_logger(), "No LED reading had error: %s", led_off_spectral_data_result->message.c_str());
        led_off_spectral_data.success = false;
        response->message = "No LED reading had error: " + led_off_spectral_data_result->message;
    }
    else
    {
        led_off_spectral_data.success = true;
        if (!led_off_spectral_data_result->success || led_off_spectral_data_result->analog_saturation || led_off_spectral_data_result->digital_saturation || !led_off_spectral_data_result->data_valid)
        {
            RCLCPP_WARN(this->get_logger(), "No LED reading should be redone!");
            led_off_spectral_data.redo_reading = true;
        }
        else
        {
            led_off_spectral_data.redo_reading = false;
        }

        led_off_spectral_data.f1_405nm = led_off_spectral_data_result->f1_405nm;
        led_off_spectral_data.f2_425nm = led_off_spectral_data_result->f2_425nm;
        led_off_spectral_data.fz_450nm = led_off_spectral_data_result->fz_450nm;
        led_off_spectral_data.f3_475nm = led_off_spectral_data_result->f3_475nm;
        led_off_spectral_data.f4_515nm = led_off_spectral_data_result->f4_515nm;
        led_off_spectral_data.f5_550nm = led_off_spectral_data_result->f5_550nm;
        led_off_spectral_data.fy_555nm = led_off_spectral_data_result->fy_555nm;
        led_off_spectral_data.fxl_600nm = led_off_spectral_data_result->fxl_600nm;
        led_off_spectral_data.f6_640nm = led_off_spectral_data_result->f6_640nm;
        led_off_spectral_data.f7_690nm = led_off_spectral_data_result->f7_690nm;
        led_off_spectral_data.f8_745nm = led_off_spectral_data_result->f8_745nm;
        led_off_spectral_data.nir_855nm = led_off_spectral_data_result->nir_855nm;
        led_off_spectral_data.vis_clear = led_off_spectral_data_result->vis_clear;
        led_off_spectral_data.fd_flicker = led_off_spectral_data_result->fd_flicker;
    }
    response->no_led_reading = led_off_spectral_data;

    // Turn white LED on
    auto request_led_on = std::make_shared<std_srvs::srv::SetBool::Request>();
    request_led_on->data = true;
    while (!_led_client->wait_for_service(100ms))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted by error while waiting for led service, exiting");
        }
        RCLCPP_INFO(this->get_logger(), "Timed out waiting for led service, waiting again");
    }
    auto led_on_result = _led_client->async_send_request(request_led_on);
    led_on_result.wait();

    while (!_spectral_data_client->wait_for_service(100ms))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted by error while waiting for spectral data service, exiting");
        }
        RCLCPP_INFO(this->get_logger(), "Timed out waiting for spectral data service, waiting again");
    }

    auto led_white_spectral_data_future = _spectral_data_client->async_send_request(std::make_shared<perseus_interfaces::srv::GetSpectralData::Request>());
    led_white_spectral_data_future.wait();
    auto led_white_spectral_data_result = led_white_spectral_data_future.get();
    perseus_interfaces::msg::As7343Data led_white_spectral_data;

    if (!led_white_spectral_data_result->success)
    {
        RCLCPP_ERROR(this->get_logger(), "White LED reading had error: %s", led_white_spectral_data_result->message.c_str());
        led_white_spectral_data.success = false;
        response->message = response->message + ", White LED reading had error: " + led_white_spectral_data_result->message;
    }
    else
    {
        led_white_spectral_data.success = true;
        if (!led_white_spectral_data_result->success || led_white_spectral_data_result->analog_saturation || led_off_spectral_data_result->digital_saturation || !led_off_spectral_data_result->data_valid)
        {
            RCLCPP_WARN(this->get_logger(), "White LED reading should be redone!");
            led_white_spectral_data.redo_reading = true;
        }
        else
        {
            led_white_spectral_data.redo_reading = false;
        }

        led_white_spectral_data.f1_405nm = led_white_spectral_data_result->f1_405nm;
        led_white_spectral_data.f2_425nm = led_white_spectral_data_result->f2_425nm;
        led_white_spectral_data.fz_450nm = led_white_spectral_data_result->fz_450nm;
        led_white_spectral_data.f3_475nm = led_white_spectral_data_result->f3_475nm;
        led_white_spectral_data.f4_515nm = led_white_spectral_data_result->f4_515nm;
        led_white_spectral_data.f5_550nm = led_white_spectral_data_result->f5_550nm;
        led_white_spectral_data.fy_555nm = led_white_spectral_data_result->fy_555nm;
        led_white_spectral_data.fxl_600nm = led_white_spectral_data_result->fxl_600nm;
        led_white_spectral_data.f6_640nm = led_white_spectral_data_result->f6_640nm;
        led_white_spectral_data.f7_690nm = led_white_spectral_data_result->f7_690nm;
        led_white_spectral_data.f8_745nm = led_white_spectral_data_result->f8_745nm;
        led_white_spectral_data.nir_855nm = led_white_spectral_data_result->nir_855nm;
        led_white_spectral_data.vis_clear = led_white_spectral_data_result->vis_clear;
        led_white_spectral_data.fd_flicker = led_white_spectral_data_result->fd_flicker;
    }
    response->white_led_reading = led_white_spectral_data;

    // Turn off white LED
    while (!_led_client->wait_for_service(100ms))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted by error while waiting for led service, exiting");
        }
        RCLCPP_INFO(this->get_logger(), "Timed out waiting for led service, waiting again");
    }
    led_off_result = _led_client->async_send_request(request_led_off);
    led_off_result.wait();

    // Turn on UV LED
    gpioWrite(_uv_led_pin, 1);

    while (!_spectral_data_client->wait_for_service(100ms))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted by error while waiting for spectral data service, exiting");
        }
        RCLCPP_INFO(this->get_logger(), "Timed out waiting for spectral data service, waiting again");
    }

    auto led_uv_spectral_data_future = _spectral_data_client->async_send_request(std::make_shared<perseus_interfaces::srv::GetSpectralData::Request>());
    led_uv_spectral_data_future.wait();
    auto led_uv_spectral_data_result = led_uv_spectral_data_future.get();
    perseus_interfaces::msg::As7343Data led_uv_spectral_data;

    if (!led_uv_spectral_data_result->success)
    {
        RCLCPP_ERROR(this->get_logger(), "UV LED reading had error: %s", led_uv_spectral_data_result->message.c_str());
        led_uv_spectral_data.success = false;
        response->message = response->message + ", UV LED reading had error: " + led_uv_spectral_data_result->message;
    }
    else
    {
        led_uv_spectral_data.success = true;
        if (!led_uv_spectral_data_result->success || led_uv_spectral_data_result->analog_saturation || led_off_spectral_data_result->digital_saturation || !led_off_spectral_data_result->data_valid)
        {
            RCLCPP_WARN(this->get_logger(), "UV LED reading should be redone!");
            led_uv_spectral_data.redo_reading = true;
        }
        else
        {
            led_uv_spectral_data.redo_reading = false;
        }

        led_uv_spectral_data.f1_405nm = led_uv_spectral_data_result->f1_405nm;
        led_uv_spectral_data.f2_425nm = led_uv_spectral_data_result->f2_425nm;
        led_uv_spectral_data.fz_450nm = led_uv_spectral_data_result->fz_450nm;
        led_uv_spectral_data.f3_475nm = led_uv_spectral_data_result->f3_475nm;
        led_uv_spectral_data.f4_515nm = led_uv_spectral_data_result->f4_515nm;
        led_uv_spectral_data.f5_550nm = led_uv_spectral_data_result->f5_550nm;
        led_uv_spectral_data.fy_555nm = led_uv_spectral_data_result->fy_555nm;
        led_uv_spectral_data.fxl_600nm = led_uv_spectral_data_result->fxl_600nm;
        led_uv_spectral_data.f6_640nm = led_uv_spectral_data_result->f6_640nm;
        led_uv_spectral_data.f7_690nm = led_uv_spectral_data_result->f7_690nm;
        led_uv_spectral_data.f8_745nm = led_uv_spectral_data_result->f8_745nm;
        led_uv_spectral_data.nir_855nm = led_uv_spectral_data_result->nir_855nm;
        led_uv_spectral_data.vis_clear = led_uv_spectral_data_result->vis_clear;
        led_uv_spectral_data.fd_flicker = led_uv_spectral_data_result->fd_flicker;
    }
    response->uv_led_reading = led_uv_spectral_data;
}

IlmeniteStation::~IlmeniteStation()
{
    gpioTerminate();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<IlmeniteStation>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running ilmenite station: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
