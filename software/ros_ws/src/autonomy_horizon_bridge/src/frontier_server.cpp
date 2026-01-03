#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "perseus_msgs/srv/get_frontier_goal.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class FrontierServer : public rclcpp::Node
{
public:
  FrontierServer()
  : Node("frontier_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(),
      std::bind(&FrontierServer::mapCallback, this, _1));

    service_ = this->create_service<perseus_msgs::srv::GetFrontierGoal>(
      "/horizon_bridge/get_frontier_goal",
      std::bind(&FrontierServer::handleRequest, this, _1, _2));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/horizon_bridge/frontiers", 1);

    RCLCPP_INFO(this->get_logger(), "FrontierServer started.");
  }

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Service<perseus_msgs::srv::GetFrontierGoal>::SharedPtr service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
  }

  bool isFrontierCell(int x, int y)
  {
    int w = map_->info.width;
    int h = map_->info.height;
    int idx = y * w + x;

    if (map_->data[idx] != 0) return false;

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < 4; ++i) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (map_->data[ny * w + nx] < 0) return true;
    }
    return false;
  }

  void handleRequest(
    const std::shared_ptr<perseus_msgs::srv::GetFrontierGoal::Request> req,
    std::shared_ptr<perseus_msgs::srv::GetFrontierGoal::Response> res)
  {
    if (!map_) {
      res->success = false;
      res->message = "No map received yet";
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (...) {
      res->success = false;
      res->message = "TF lookup failed";
      return;
    }

    double rx = tf.transform.translation.x;
    double ry = tf.transform.translation.y;

    double gx = req->original_goal.pose.position.x;
    double gy = req->original_goal.pose.position.y;

    double best_score = -1e9;
    geometry_msgs::msg::PoseStamped best_goal;

    int w = map_->info.width;
    int h = map_->info.height;
    double reso = map_->info.resolution;
    double ox = map_->info.origin.position.x;
    double oy = map_->info.origin.position.y;

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (!isFrontierCell(x, y)) continue;

        double wx = ox + (x + 0.5) * reso;
        double wy = oy + (y + 0.5) * reso;

        double dist = hypot(wx - rx, wy - ry);
        double align =
          ((wx - rx) * (gx - rx) + (wy - ry) * (gy - ry)) /
          (hypot(wx - rx, wy - ry) * hypot(gx - rx, gy - ry) + 1e-6);

        double score = align * 10.0 - dist;
        if (score > best_score) {
          best_score = score;
          best_goal.header.frame_id = "map";
          best_goal.pose.position.x = wx;
          best_goal.pose.position.y = wy;
          best_goal.pose.orientation.w = 1.0;
        }
      }
    }

    if (best_score < -1e8) {
      res->success = false;
      res->message = "No valid frontier found";
      return;
    }

    res->frontier_goal = best_goal;
    res->success = true;
    res->message = "Frontier selected";
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierServer>());
  rclcpp::shutdown();
  return 0;
}
