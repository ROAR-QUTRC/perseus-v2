#include "example_plugin.hpp"

#include <fstream>
#include <iostream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_ros2/plugins.hpp"

bool SleepAction::setGoal(Goal& goal)
{
    auto timeout = getInput<unsigned>("msec");
    goal.msec_timeout = timeout.value();
    return true;
}

BT::NodeStatus SleepAction::onResultReceived(const WrappedResult& wr)
{
    RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
                wr.result->done ? "true" : "false");
    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT::NodeStatus SleepAction::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
    return NodeStatus::FAILURE;
}

void SleepAction::onHalt()
{
    RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class SleepAction will self register with name  "SleepAction".
CreateRosNodePlugin(SleepAction, "SleepAction");

void ExportNodesToXML(const std::string& filename, int argc, char** argv)
{
    // Initialize the ROS2 node
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("sleep_client");

    std::string filename_ = ament_index_cpp::get_package_share_directory("perseus_behavior") + "/" + filename;

    // Create a BehaviorTreeFactory
    BT::BehaviorTreeFactory factory;

    RosNodeParams params;
    params.nh = nh;
    params.default_port_value = "sleep_service";

    // Register the SleepAction node
    factory.registerNodeType<SleepAction>("SleepAction", params);

    // Open the output file
    std::ofstream xml_file(filename_);
    if (!xml_file.is_open())
    {
        throw std::runtime_error("Failed to open file for writing: " + filename_);
    }

    // Write the tree nodes model to the XML file
    std::string xml_string = BT::writeTreeNodesModelXML(factory, false);
    xml_file << xml_string;

    // Close the file
    xml_file.close();

    std::cout << "Tree nodes exported successfully to " << filename_ << std::endl;
    rclcpp::shutdown();
}
