#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <std_msgs/msg/float32.hpp>

class PerseusActionServer : public BT::TreeExecutionServer
{
private:
    std::shared_ptr<BT::StdCoutLogger> logger_cout_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;

public:
    PerseusActionServer(const rclcpp::NodeOptions& options) : TreeExecutionServer(options)
    {
        // Example of what we could subscribe to
        sub_ = node()->create_subscription<std_msgs::msg::Float32>(
            "battery_level", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                globalBlackboard()->set("battery_level", msg->data);
            });
    }

    void onTreeCreated(BT::Tree& tree) override
    {
        logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
    }

    std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) override
    {
        logger_cout_.reset();
        return std::nullopt;
    }
    
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<PerseusActionServer>(options);

  // This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
}