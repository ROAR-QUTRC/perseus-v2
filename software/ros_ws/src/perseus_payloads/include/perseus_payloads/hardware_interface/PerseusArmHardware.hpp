#ifndef PERSEUS_PAYLOADS__PERSEUS_ARM_HARDWARE_HPP_
#define PERSEUS_PAYLOADS__PERSEUS_ARM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "perseus_msgs/msg/arm_control.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PerseusArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PerseusArmHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> _hw_commands;
  std::vector<double> _hw_commands_velocity;
  std::vector<double> _hw_states_position;
  std::vector<double> _hw_states_velocity;
  
  // Mapping for easier access
  std::map<std::string, size_t> _joint_index_map;


  rclcpp::Node::SharedPtr _node;
  
  // Publishers
  rclcpp::Publisher<perseus_msgs::msg::ArmControl>::SharedPtr _rsbl_publisher;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _rsbl_status_subscriber;
  
  std::vector<double> _latest_rsbl_status;
  
  void _rsbl_status_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

#endif 
