#include "space_resources/ilmenite/main.hpp"
#include <memory>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<space_resources::SpectralSensor>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
