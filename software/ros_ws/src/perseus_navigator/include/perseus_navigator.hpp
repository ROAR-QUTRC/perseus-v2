#ifndef PERSEUS_NAVIGATOR_HPP_
#define PERSEUS_NAVIGATOR_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace perseus_navigator
{

    class PerseusNavigator : public nav2_core::BehaviorTreeNavigator<nav2_msgs::action::NavigateToPose>
    {
    public:
        using ActionT = nav2_msgs::action::NavigateToPose;

        /**
         * @brief Construct a new Perseus Navigator object
         *
         */
        PerseusNavigator() : BehaviorTreeNavigator()
        {
        }

        /**
         * @brief A configure state transition to configure navigator's state
         *
         * @param node the lifecylec node
         * @param odom_smoother Object to get current smoothed robot's speed
         */
        bool configure(
            rclcpp_lifecycle::LifecycleNode::WeakPtr node,
            std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

        /**
         * @brief A cleanup state transition to remove memory allocated
         *
         */
        bool cleanup() override;

        /**
         * @brief Get action name for this navigator
         * @return string Name of action server
         */
        std::string getName() override { return std::string("perseus_navigator"); }

        /**
         * @brief Get the Default B T Filepath object
         *
         * @param node to the lifecycle nodenode
         * @return std::string Filepath to default XML
         */
        std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

    protected:
        /**
         * @brief A callback to be called when a new goal is received by the BT action server
         *
         * @param goal Action template's goal message
         * @return bool if goal was received successfully to be processed
         */
        bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

        /**
         * @brief A callback that defines execution that happens on one iteration through the BT
         *
         */
        void onLoop() override;

        /**
         * @brief A callback that is called when a preempt is requested
         *
         * @param goal
         */
        void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

        /**
         * @brief A callback that is called when the action is completed, can fill in action result message or indicate that this action is done.
         *
         * @param result Action template result message to populate
         * @param final_bt_status Resulting status of the behavior tree execution that may be referenced while populating the result
         */
        void goalCompleted(
            typename ActionT::Result::SharedPtr result,
            const nav2_behavior_tree::BtStatus final_bt_status) override;

        rclcpp::Time start_time_;

        rclcpp_action::Client<ActionT>::SharedPtr self_client_;
        std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
    };

}  // namespace perseus_navigator

#endif  // PERSEUS_NAVIGATOR_HPP_