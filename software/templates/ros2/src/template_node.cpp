#include "template_node.hpp"

// Called on startup of node
TemplateNode::TemplateNode(const rclcpp::NodeOptions& options)
    : Node("template_node", options)
{
    _templateSubscription =
        this->create_subscription<std_msgs::msg::UInt8>("subscribed_topic_name", 10, std::bind(&TemplateNode::_templateCallback, this, std::placeholders::_1));
    _templateTimer = this->create_wall_timer(TIMER_PERIOD, std::bind(&TemplateNode::_timerCallback, this));
    _templatePublisher = this->create_publisher<std_msgs::msg::UInt16>("publishing_topic_name", 10);

    RCLCPP_INFO(this->get_logger(), "Template node initialized");
}

// Called whenever a message is received over subscribed_topic_name
void TemplateNode::_templateCallback(const std_msgs::msg::UInt8 msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Callback function - Received something from subscribed_topic_name");
    _lastMsgReceived = msg.data;
    std_msgs::msg::UInt16 UInt16Msg;
    UInt16Msg.data = uint16_t(msg.data);
    _templatePublisher->publish(UInt16Msg);
    RCLCPP_INFO(this->get_logger(), "Sent received data to publishing_topic_name");
}

// Runs every TIMER_PERIOD (100ms)
void TemplateNode::_timerCallback(void)
{
    RCLCPP_INFO(this->get_logger(), "Last data received was %d", _lastMsgReceived);
}

// Initialises everything, and shuts down the node after it's finished
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions nodeOptions{};
    try
    {
        nodeOptions.allow_undeclared_parameters(true);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Template Node");
        auto node = std::make_shared<TemplateNode>(nodeOptions);
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running Template Node: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
