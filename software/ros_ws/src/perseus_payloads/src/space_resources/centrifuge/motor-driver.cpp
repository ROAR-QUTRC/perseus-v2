#include "space_resources/centrifuge/centrifuge_driver.hpp"

namespace space_resources
{

    CentrifugeDriver::CentrifugeDriver(const rclcpp::NodeOptions& options)
        : Node("centrifuge_driver", options)
    {
        RCLCPP_INFO(this->get_logger(), "Centrifuge driver node started");
    }

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<space_resources::CentrifugeDriver>());
    rclcpp::shutdown();
    return 0;
}
