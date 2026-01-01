#include <cmath>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "perseus_autonomy_interfaces/srv/cancel_waypoints.hpp"
#include "perseus_autonomy_interfaces/srv/run_waypoints.hpp"

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
        auto is_space = [](unsigned char c)
        { return std::isspace(c); };
        while (!s.empty() && is_space(s.front())) s.erase(s.begin());
        while (!s.empty() && is_space(s.back())) s.pop_back();
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
        action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "/navigate_through_poses");

        srv_run_ = create_service<perseus_autonomy_interfaces::srv::RunWaypoints>(
            "/autonomy/run_waypoints",
            std::bind(&Nav2WaypointsBridge::on_run, this, std::placeholders::_1, std::placeholders::_2));

        srv_cancel_ = create_service<perseus_autonomy_interfaces::srv::CancelWaypoints>(
            "/autonomy/cancel_waypoints",
            std::bind(&Nav2WaypointsBridge::on_cancel, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Ready: /autonomy/run_waypoints, /autonomy/cancel_waypoints");
    }

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
    GoalHandleNav::SharedPtr active_goal_;

    rclcpp::Service<perseus_autonomy_interfaces::srv::RunWaypoints>::SharedPtr srv_run_;
    rclcpp::Service<perseus_autonomy_interfaces::srv::CancelWaypoints>::SharedPtr srv_cancel_;

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

            auto wps = load_waypoints_yaml(req->yaml_path);

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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2WaypointsBridge>());
    rclcpp::shutdown();
    return 0;
}
