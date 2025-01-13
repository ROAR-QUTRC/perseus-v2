#include <std_msgs/msg/string.hpp>

#include "behaviortree_cpp/loggers/groot2_protocol.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

using namespace BT;

class ReceiveString : public RosTopicSubNode<std_msgs::msg::String>
{
public:
    ReceiveString(const std::string& name, const NodeConfig& conf,
                  const RosNodeParams& params)
        : RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override
    {
        if (last_msg)  // empty if no new message received, since the last tick
        {
            RCLCPP_INFO(logger(), "[%s] new message: %s", name().c_str(),
                        last_msg->data.c_str());
        }
        return NodeStatus::SUCCESS;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("subscriber_test");

    BehaviorTreeFactory factory;

    BT::RosNodeParams params;
    params.nh = nh;
    params.default_port_value = "btcpp_string";
    factory.registerNodeType<ReceiveString>("ReceiveString", params);

    auto tree = factory.createTreeFromFile("../behavior_trees/perseus_bt.xml");

    const unsigned port = 1667;
    BT::Groot2Publisher publisher(tree, port);

    while (rclcpp::ok())
    {
        tree.tickWhileRunning();
    }

    return 0;
}