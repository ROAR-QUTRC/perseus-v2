#include "space_resources/centrifuge/centrifuge_driver.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentrifugeDriver>());
    rclcpp::shutdown();
    return 0;
}
