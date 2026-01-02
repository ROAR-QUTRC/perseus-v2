#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <optional>
#include <cmath>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/serialization.hpp"

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// bt services and messages
#include "perseus_autonomy_interfaces/srv/run_waypoints.hpp"
#include "perseus_autonomy_interfaces/srv/cancel_waypoints.hpp"
#include "perseus_autonomy_interfaces/msg/navigation_info.hpp"
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

struct Waypoint
{
    std::string name;
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

// Minimal YAML parser for:
// waypoints:
//  - name: WP1
//    x: 1.2
//    y: -0.5
//    yaw: 1.57
static std::vector<Waypoint> load_waypoints_yaml(const std::string& path)
{
    std::ifstream in(path);
    if (!in.is_open())
    {
        throw std::runtime_error("Cannot open YAML file: " + path);
    }

    auto trim = [](std::string s)
    {
        auto is_space = [](unsigned char c) { return std::isspace(c); };
        while (!s.empty() && is_space(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
        while (!s.empty() && is_space(static_cast<unsigned char>(s.back()))) s.pop_back();
        return s;
    };

    auto parse_kv = [&](const std::string& s, std::string& k, std::string& v) -> bool
    {
        auto pos = s.find(':');
        if (pos == std::string::npos)
            return false;
        k = trim(s.substr(0, pos));
        v = trim(s.substr(pos + 1));
        return true;
    };

    std::vector<Waypoint> out;
    std::string line;
    std::optional<Waypoint> cur;

    while (std::getline(in, line))
    {
        line = trim(line);
        if (line.empty() || line.rfind("#", 0) == 0)
            continue;
        if (line == "waypoints:")
            continue;

        if (line.rfind("- ", 0) == 0)
        {
            if (cur.has_value())
                out.push_back(*cur);
            cur = Waypoint{};
            std::string rest = trim(line.substr(2));
            if (!rest.empty())
            {
                std::string k, v;
                if (parse_kv(rest, k, v))
                {
                    if (k == "name")
                        cur->name = v;
                    else if (k == "x")
                        cur->x = std::stod(v);
                    else if (k == "y")
                        cur->y = std::stod(v);
                    else if (k == "yaw")
                        cur->yaw = std::stod(v);
                }
            }
            continue;
        }

        if (cur.has_value())
        {
            std::string k, v;
            if (parse_kv(line, k, v))
            {
                if (k == "name")
                    cur->name = v;
                else if (k == "x")
                    cur->x = std::stod(v);
                else if (k == "y")
                    cur->y = std::stod(v);
                else if (k == "yaw")
                    cur->yaw = std::stod(v);
            }
        }
    }

    if (cur.has_value())
        out.push_back(*cur);

    for (size_t i = 0; i < out.size(); i++)
    {
        if (out[i].name.empty())
            out[i].name = "WP" + std::to_string(i + 1);
    }
    if (out.empty())
    {
        throw std::runtime_error("No waypoints found in YAML: " + path);
    }
    return out;
}

class Nav2WaypointsBridge : public rclcpp::Node
{
public:
    Nav2WaypointsBridge()
        : Node("nav2_waypoints_bridge")
    {
        // Directory where YAML files live on the robot/PC running this node.
        // IMPORTANT: UI should send only a filename (e.g. "waypoints_simulation.yaml").
        waypoints_dir_ = this->declare_parameter<std::string>("waypoints_dir", "");

        // Customizable feedback topic name
        feedback_topic_name_ = this->declare_parameter<std::string>("feedback_topic", "/autonomy/navigation_info");

        action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "/navigate_through_poses");

        // Publisher for navigation info
        nav_info_pub_ = this->create_publisher<perseus_autonomy_interfaces::msg::NavigationInfo>(
            feedback_topic_name_, 10);

        // Subscribe to the feedback topic using a generic subscription with a lambda
        // The feedback message contains the navigation data we need
        auto generic_callback = [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            // Store the serialized message for later deserialization
            this->last_serialized_feedback_ = msg;
            // Update the timestamp to track when we last received feedback
            this->last_feedback_time_ = this->now();
        };
        
        feedback_sub_ = this->create_generic_subscription(
            "/navigate_through_poses/_action/feedback",
            "nav2_msgs/action/NavigateThroughPoses_FeedbackMessage",
            rclcpp::SensorDataQoS(),
            generic_callback);

        // Timer to publish navigation info periodically
        nav_info_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Nav2WaypointsBridge::publish_nav_info, this));

        // Initialize last_feedback_time_ to now so we don't start in an inactive state
        // last_feedback_time_ = this->now();

        srv_run_ = create_service<perseus_autonomy_interfaces::srv::RunWaypoints>(
            "/autonomy/run_waypoints",
            std::bind(&Nav2WaypointsBridge::on_run, this, std::placeholders::_1, std::placeholders::_2));

        srv_cancel_ = create_service<perseus_autonomy_interfaces::srv::CancelWaypoints>(
            "/autonomy/cancel_waypoints",
            std::bind(&Nav2WaypointsBridge::on_cancel, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Ready: /autonomy/run_waypoints, /autonomy/cancel_waypoints");
        RCLCPP_INFO(get_logger(), "Feedback topic: '%s'", feedback_topic_name_.c_str());
        RCLCPP_INFO(get_logger(), "waypoints_dir param: '%s'", waypoints_dir_.c_str());
    }

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
    GoalHandleNav::SharedPtr active_goal_;

    rclcpp::Publisher<perseus_autonomy_interfaces::msg::NavigationInfo>::SharedPtr nav_info_pub_;
    rclcpp::TimerBase::SharedPtr nav_info_timer_;
    std::shared_ptr<rclcpp::GenericSubscription> feedback_sub_;
    std::shared_ptr<rclcpp::SerializedMessage> last_serialized_feedback_;

    // Current navigation feedback data
    nav2_msgs::action::NavigateThroughPoses::Feedback current_feedback_;
    builtin_interfaces::msg::Time last_feedback_time_;

    rclcpp::Service<perseus_autonomy_interfaces::srv::RunWaypoints>::SharedPtr srv_run_;
    rclcpp::Service<perseus_autonomy_interfaces::srv::CancelWaypoints>::SharedPtr srv_cancel_;

    std::string waypoints_dir_;
    std::string feedback_topic_name_;

    // Resolve request yaml_path into a full filesystem path.
    // - If req sends "/waypoints_simulation.yaml" => treat as "waypoints_simulation.yaml"
    // - If req sends "waypoints_simulation.yaml"  => use it as-is
    // - If waypoints_dir is set => prepend it
    std::string resolve_yaml_path(const std::string& req_path) const
    {
        std::string p = req_path;

        // Strip leading '/' so the request cannot accidentally become "absolute from root"
        while (!p.empty() && p.front() == '/')
            p.erase(p.begin());

        if (p.empty())
            p = "waypoints.yaml";

        if (waypoints_dir_.empty())
        {
            // No directory configured: fall back to current working directory (not recommended)
            return p;
        }

        std::string dir = waypoints_dir_;
        // ensure trailing slash
        if (!dir.empty() && dir.back() != '/')
            dir.push_back('/');

        return dir + p;
    }

    void on_run(
        const std::shared_ptr<perseus_autonomy_interfaces::srv::RunWaypoints::Request> req,
        std::shared_ptr<perseus_autonomy_interfaces::srv::RunWaypoints::Response> resp)
    {
        try
        {
            if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
            {
                resp->success = false;
                resp->message = "Nav2 action server /navigate_through_poses not available";
                return;
            }

            if (active_goal_)
            {
                action_client_->async_cancel_goal(active_goal_);
                active_goal_.reset();
            }

            const std::string yaml_full_path = resolve_yaml_path(req->yaml_path);
            RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s (req: '%s')",
                        yaml_full_path.c_str(), req->yaml_path.c_str());

            auto wps = load_waypoints_yaml(yaml_full_path);

            NavigateThroughPoses::Goal goal;
            goal.poses.reserve(wps.size());

            for (const auto& wp : wps)
            {
                geometry_msgs::msg::PoseStamped ps;
                ps.header.frame_id = "map";
                ps.pose.position.x = wp.x;
                ps.pose.position.y = wp.y;
                ps.pose.position.z = 0.0;
                ps.pose.orientation = yaw_to_quat(wp.yaw);
                goal.poses.push_back(ps);
            }

            rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions opts;

            opts.goal_response_callback =
                [this](const GoalHandleNav::SharedPtr goal_handle)
            {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal rejected by Nav2");
                    return;
                }
                this->active_goal_ = goal_handle;
                RCLCPP_INFO(this->get_logger(), "Goal accepted");
            };

            opts.result_callback =
                [this](const GoalHandleNav::WrappedResult& /*result*/)
            {
                RCLCPP_INFO(this->get_logger(), "Goal finished (result received)");
                this->active_goal_.reset();
            };

            opts.feedback_callback =
                [this](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
            {
                if (!feedback)
                    return;
                // Store the feedback; the timer will publish it with proper navigation_active flag
                this->current_feedback_ = *feedback;
                this->last_feedback_time_ = this->now();
            };

            action_client_->async_send_goal(goal, opts);

            resp->success = true;
            resp->message = "Goal sent to /navigate_through_poses";
        }
        catch (const std::exception& e)
        {
            resp->success = false;
            resp->message = e.what();
        }
    }

    void on_cancel(
        const std::shared_ptr<perseus_autonomy_interfaces::srv::CancelWaypoints::Request>,
        std::shared_ptr<perseus_autonomy_interfaces::srv::CancelWaypoints::Response> resp)
    {
        if (!active_goal_)
        {
            resp->success = true;
            resp->message = "No active goal";
            return;
        }

        action_client_->async_cancel_goal(active_goal_);
        active_goal_.reset();
        resp->success = true;
        resp->message = "Cancel requested";
    }

    void publish_nav_info()
    {
        // Try to deserialize the feedback if we have a serialized message
        if (last_serialized_feedback_)
        {
            try
            {
                // Deserialize the FeedbackMessage which wraps the actual Feedback
                rclcpp::Serialization<nav2_msgs::action::NavigateThroughPoses_FeedbackMessage> serialization;
                nav2_msgs::action::NavigateThroughPoses_FeedbackMessage feedback_msg;
                serialization.deserialize_message(last_serialized_feedback_.get(), &feedback_msg);

                // Extract the feedback struct
                current_feedback_ = feedback_msg.feedback;
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Failed to deserialize feedback: %s", e.what());
            }
        }

        // Create and publish the navigation info message
        auto nav_info = std::make_unique<perseus_autonomy_interfaces::msg::NavigationInfo>();
        nav_info->current_pose = current_feedback_.current_pose;
        nav_info->navigation_time = current_feedback_.navigation_time;
        nav_info->estimated_time_remaining = current_feedback_.estimated_time_remaining;
        nav_info->number_of_recoveries = current_feedback_.number_of_recoveries;
        nav_info->distance_remaining = current_feedback_.distance_remaining;
        nav_info->number_of_poses_remaining = current_feedback_.number_of_poses_remaining;
        
        // Check if navigation is active: we need both an active goal AND recent feedback
        auto now = this->now();
        auto time_since_feedback = (now - last_feedback_time_).seconds();
        // Active only if we have an active goal AND received feedback within last 2 seconds
        nav_info->navigation_active = (active_goal_ != nullptr) || (time_since_feedback < 0.2);
        RCLCPP_INFO(this->get_logger(), "Time since last feedback: %.2f seconds, active_goal: %s", 
                    time_since_feedback, (active_goal_ != nullptr) ? "yes" : "no");
        nav_info_pub_->publish(std::move(nav_info));
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2WaypointsBridge>());
    rclcpp::shutdown();
    return 0;
}
