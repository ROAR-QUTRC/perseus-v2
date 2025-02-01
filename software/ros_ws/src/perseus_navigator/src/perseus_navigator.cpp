#include "perseus_navigator.hpp"

namespace perseus_navigator
{

    bool PerseusNavigator::configure(
        rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
        std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
    {
        // Configuring private variables

        start_time_ = rclcpp::Time(0);
        odom_smoother_ = odom_smoother;

        auto node = parent_node.lock();
        self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

        return true;
    }

    std::string PerseusNavigator::getDefaultBTFilepath(
        rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
    {
        std::string default_bt_filename;
        auto node = parent_node.lock();

        // If no default bt has been given go to default location
        // Which is /behavior_trees/perseus_bt.xml
        if (!node->has_parameter("default_bt"))
        {
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("perseus_navigator");
            node->declare_parameter<std::string>(
                "default_bt",
                pkg_share_dir + "/behavior_trees/perseus_bt.xml");
        }

        // Get the bt filename
        node->get_parameter("default_bt", default_bt_filename);
        return default_bt_filename;
    }

    bool PerseusNavigator::cleanup()
    {
        self_client_.reset();
        return true;
    }

    bool PerseusNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
    {
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("perseus_navigator");
        auto bt_filename = pkg_share_dir + "/behavior_trees/" + goal->behavior_tree;

        if (!bt_action_server_->loadBehaviorTree(bt_filename))
        {
            RCLCPP_ERROR(
                logger_, "BT file not found: %s. Navigation canceled.",
                bt_filename.c_str());
            return false;
        }

        start_time_ = clock_->now();

        // TODO: Decide how to start goals
        return true;
    }

    void PerseusNavigator::goalCompleted(
        typename ActionT::Result::SharedPtr result,
        const nav2_behavior_tree::BtStatus final_bt_status)
    {
        if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED)
        {
            result->total_elapsed_time = clock_->now() - start_time_;
        }
    }

    void PerseusNavigator::onLoop()
    {
        auto feedback_msg = std::make_shared<ActionT::Feedback>();

        feedback_msg->navigation_time = clock_->now() - start_time_;
        bt_action_server_->publishFeedback(feedback_msg);
    }

    void PerseusNavigator::onPreempt(ActionT::Goal::ConstSharedPtr /*goal*/)
    {
        RCLCPP_INFO(logger_, "Received goal preemption request");
    }
}  // namespace perseus_navigator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(perseus_navigator::PerseusNavigator, nav2_core::NavigatorBase)