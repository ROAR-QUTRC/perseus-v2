#include "system_status.hpp"


SystemStatus::SystemStatus(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node ("system_status", options)
{
    // initialise the CAN interface
    auto& interface = TwaiInterface::getInstance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                 addressing::filter_t{
                                                     .address = 0x01000000,
                                                     .mask = hi_can::addressing::DEVICE_MASK,
                                                 });
    packetManager.emplace(interface);
    packetManager->addGroup(computeBus);
    packetManager->addGroup(driveBus);
    packetManager->addGroup(auxBus);
    packetManager->addGroup(spareBus);

    _callBackTimer = this->create_wall_timer(
        std::chrono::milliseconds(callBackPeriod_ms),
        std::bind(&SystemStatus::_publishSystemStatusCallBack, this));

    
}

void SystemStatus::_publishSystemStatusCallBack (const system_status_msgs::msg::SystemStatus::SharedPtr msg){
    
}
