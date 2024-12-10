#pragma once

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hi_can.hpp>
#include <optional>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

namespace perseus_hardware
{
    class VescSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(VescSystemHardware)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        auto& get_logger() const { return *_logger; }
        auto get_clock() const { return _clock; }
        auto& get_info() const { return info_; }

    private:
        std::optional<hi_can::PacketManager> _packetManager;

        std::vector<double> _commandSpeeds;
        std::vector<double> _realSpeeds;
        std::vector<double> _realPositions;

        std::shared_ptr<rclcpp::Logger> _logger;
        rclcpp::Clock::SharedPtr _clock;
    };
}