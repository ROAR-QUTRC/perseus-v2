#include <behaviortree_ros2/tree_execution_server.hpp>

#include "behaviortree_cpp/loggers/bt_cout_logger.h"

class PerseusBTServer : public BT::TreeExecutionServer
{
private:
    std::shared_ptr<BT::StdCoutLogger> logger_cout_;

public:
    explicit PerseusBTServer(const rclcpp::NodeOptions& options) : BT::TreeExecutionServer(std::make_shared<rclcpp::Node>("perseus_bt_action_server", options))
    {
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

    std::string custom_action_name = "bt_execution";

    rclcpp::NodeOptions options;
    options.parameter_overrides().push_back({"action_name", custom_action_name});
    auto action_server = std::make_shared<PerseusBTServer>(options);

    // This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
    // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                  std::chrono::milliseconds(250));
    exec.add_node(action_server->node());
    exec.spin();
    exec.remove_node(action_server->node());

    rclcpp::shutdown();
}