#pragma once

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hi_can_raw.hpp>
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
        virtual ~VescSystemHardware();

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // note: using std::optional so we can delay initialisation until on_configure
        std::optional<hi_can::RawCanInterface> _can_interface;
        std::optional<hi_can::PacketManager> _packet_manager;

        /// @brief The last time we tried to transmit and had an error
        /// @details Used to throttle transmissions (and error messages) so we don't build up a massive transmit queue when the bus comes online again
        rclcpp::Time _last_transmit_error;

        std::vector<hi_can::parameters::drive::vesc::VescParameterGroup> _parameter_groups;
        std::vector<unsigned long> _vesc_ids;
        std::vector<double> _command_speeds;
        std::vector<double> _real_speeds;
        std::vector<double> _real_positions;
        std::vector<double> _real_currents;
        std::vector<double> _real_temperatures;
    };
}