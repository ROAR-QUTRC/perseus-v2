#include "perseus_lite_hud/hud_node.hpp"

#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace perseus_lite_hud {

HudOverlayNode::HudOverlayNode(const rclcpp::NodeOptions &options)
    : Node("hud_overlay_node", options) {
  _initialize_parameters();

  // Create HUD elements
  BearingCompassConfig compass_cfg;
  compass_cfg.width = this->declare_parameter("compass.width", 400);
  compass_cfg.height = this->declare_parameter("compass.height", 40);
  compass_cfg.degrees_visible =
      this->declare_parameter("compass.degrees_visible", 120);
  compass_cfg.opacity = this->declare_parameter("compass.opacity", 0.7);
  compass_ = std::make_shared<BearingCompass>(compass_cfg);

  PipMapConfig map_cfg;
  map_cfg.size = this->declare_parameter("pip_map.size", 180);
  map_cfg.radius_m = this->declare_parameter("pip_map.radius_m", 5.0);
  map_cfg.opacity = this->declare_parameter("pip_map.opacity", 0.8);
  map_cfg.margin = this->declare_parameter("pip_map.margin", 10);
  pip_map_ = std::make_shared<PipMap>(map_cfg);

  LidarProximityConfig lidar_cfg;
  lidar_cfg.danger_dist =
      this->declare_parameter("lidar.danger_dist", 0.5);
  lidar_cfg.warning_dist =
      this->declare_parameter("lidar.warning_dist", 1.5);
  lidar_cfg.clear_dist =
      this->declare_parameter("lidar.clear_dist", 3.0);
  lidar_cfg.bar_thickness =
      this->declare_parameter("lidar.bar_thickness", 8);
  lidar_cfg.opacity = this->declare_parameter("lidar.opacity", 0.6);
  lidar_cfg.flash_rate_hz =
      this->declare_parameter("lidar.flash_rate_hz", 4.0);
  lidar_prox_ = std::make_shared<LidarProximity>(lidar_cfg);

  VelocityGaugeConfig vel_cfg;
  vel_cfg.width = this->declare_parameter("velocity.width", 60);
  vel_cfg.height = this->declare_parameter("velocity.height", 160);
  vel_cfg.max_linear =
      this->declare_parameter("velocity.max_linear", 1.0);
  vel_cfg.max_angular =
      this->declare_parameter("velocity.max_angular", 2.0);
  vel_cfg.opacity = this->declare_parameter("velocity.opacity", 0.7);
  vel_cfg.margin = this->declare_parameter("velocity.margin", 10);
  velocity_gauge_ = std::make_shared<VelocityGauge>(vel_cfg);

  OdometerConfig odo_cfg;
  odo_cfg.opacity = this->declare_parameter("odometer.opacity", 0.8);
  odo_cfg.digit_width =
      this->declare_parameter("odometer.digit_width", 28);
  odo_cfg.digit_height =
      this->declare_parameter("odometer.digit_height", 40);
  odometer_ = std::make_shared<Odometer>(odo_cfg);

  // Register elements with renderer
  renderer_.add_element(compass_);
  renderer_.add_element(pip_map_);
  renderer_.add_element(lidar_prox_);
  renderer_.add_element(velocity_gauge_);
  renderer_.add_element(odometer_);

  // Element enable flags
  compass_->set_enabled(this->declare_parameter("compass.enabled", true));
  pip_map_->set_enabled(this->declare_parameter("pip_map.enabled", true));
  lidar_prox_->set_enabled(this->declare_parameter("lidar.enabled", true));
  velocity_gauge_->set_enabled(
      this->declare_parameter("velocity.enabled", true));
  odometer_->set_enabled(this->declare_parameter("odometer.enabled", true));

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  _initialize_publisher();
  _initialize_subscriptions();

  RCLCPP_INFO(this->get_logger(), "HUD overlay node initialized");
}

void HudOverlayNode::_initialize_parameters() {
  map_frame_ = this->declare_parameter("map_frame", "map");
  base_frame_ = this->declare_parameter("base_frame", "base_link");
  heading_source_ = this->declare_parameter("heading_source", "odom");
}

void HudOverlayNode::_initialize_publisher() {
  image_pub_ = image_transport::create_publisher(
      this, this->declare_parameter("output_topic",
                                    std::string("/camera/image_hud")));
}

void HudOverlayNode::_initialize_subscriptions() {
  auto sensor_qos = rclcpp::SensorDataQoS();

  // Camera image
  auto camera_topic = this->declare_parameter("camera_topic",
                                              std::string("/image_raw"));
  image_sub_ = image_transport::create_subscription(
      this, camera_topic,
      std::bind(&HudOverlayNode::_image_callback, this,
                std::placeholders::_1),
      "raw", rmw_qos_profile_sensor_data);

  // Raw odometry (for odometer distance)
  auto odom_topic =
      this->declare_parameter("odom_topic", std::string("/odom"));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, sensor_qos,
      std::bind(&HudOverlayNode::_odom_callback, this,
                std::placeholders::_1));

  // Filtered odometry (heading + velocity)
  auto odom_filtered_topic = this->declare_parameter(
      "odom_filtered_topic", std::string("/odometry/filtered"));
  odom_filtered_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_filtered_topic, sensor_qos,
      std::bind(&HudOverlayNode::_odom_filtered_callback, this,
                std::placeholders::_1));

  // IMU (fallback heading)
  auto imu_topic =
      this->declare_parameter("imu_topic", std::string("/imu/data"));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos,
      std::bind(&HudOverlayNode::_imu_callback, this,
                std::placeholders::_1));

  // LiDAR scan
  auto scan_topic =
      this->declare_parameter("scan_topic", std::string("/scan"));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, sensor_qos,
      std::bind(&HudOverlayNode::_scan_callback, this,
                std::placeholders::_1));

  // Map (latched)
  auto map_qos =
      rclcpp::QoS(1).transient_local().reliable();
  auto map_topic =
      this->declare_parameter("map_topic", std::string("/map"));
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, map_qos,
      std::bind(&HudOverlayNode::_map_callback, this,
                std::placeholders::_1));

  // cmd_vel (fallback velocity)
  auto cmd_vel_topic =
      this->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(10),
      std::bind(&HudOverlayNode::_cmd_vel_callback, this,
                std::placeholders::_1));
}

void HudOverlayNode::_image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "cv_bridge exception: %s", e.what());
    return;
  }

  // Snapshot cached data under lock
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (has_heading_) {
      compass_->set_heading(heading_deg_);
    }
    if (has_velocity_) {
      velocity_gauge_->set_velocity(linear_vel_, angular_vel_);
    }
    if (has_lidar_) {
      lidar_prox_->set_sector_ranges(lidar_sectors_);
    }
    if (has_odom_) {
      odometer_->update_position(odom_x_, odom_y_);
    }
  }

  // TF lookup for PIP map
  if (pip_map_->enabled()) {
    try {
      auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_,
                                            tf2::TimePointZero);
      double yaw = _yaw_from_quaternion(
          tf.transform.rotation.x, tf.transform.rotation.y,
          tf.transform.rotation.z, tf.transform.rotation.w);
      pip_map_->set_robot_pose(tf.transform.translation.x,
                               tf.transform.translation.y, yaw);
    } catch (const tf2::TransformException &) {
      // PIP map won't render without pose — graceful degradation
    }
  }

  // Render all HUD elements
  renderer_.render_all(cv_ptr->image);

  // Publish
  image_pub_.publish(cv_ptr->toImageMsg());
}

void HudOverlayNode::_odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  has_odom_ = true;
}

void HudOverlayNode::_odom_filtered_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Heading from filtered odom (primary source)
  if (heading_source_ == "odom") {
    auto &q = msg->pose.pose.orientation;
    double yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w);
    // Convert to degrees [0, 360)
    heading_deg_ = std::fmod(yaw * 180.0 / M_PI + 360.0, 360.0);
    has_heading_ = true;
  }

  // Velocity (primary source)
  linear_vel_ = msg->twist.twist.linear.x;
  angular_vel_ = msg->twist.twist.angular.z;
  has_velocity_ = true;
}

void HudOverlayNode::_imu_callback(
    const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Heading from IMU (fallback)
  if (heading_source_ == "imu" || !has_heading_) {
    auto &q = msg->orientation;
    double yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w);
    heading_deg_ = std::fmod(yaw * 180.0 / M_PI + 360.0, 360.0);
    has_heading_ = true;
  }
}

void HudOverlayNode::_scan_callback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg) {
  auto sectors = LidarProximity::compute_sectors(
      msg->ranges.data(), msg->ranges.size(), msg->angle_min,
      msg->angle_increment);

  std::lock_guard<std::mutex> lock(data_mutex_);
  lidar_sectors_ = sectors;
  has_lidar_ = true;
}

void HudOverlayNode::_map_callback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg) {
  pip_map_->set_map(msg);
}

void HudOverlayNode::_cmd_vel_callback(
    const geometry_msgs::msg::Twist::ConstSharedPtr &msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!has_velocity_) {
    linear_vel_ = msg->linear.x;
    angular_vel_ = msg->angular.z;
    has_velocity_ = true;
  }
}

double HudOverlayNode::_yaw_from_quaternion(double x, double y, double z,
                                            double w) {
  // atan2(2(wz + xy), 1 - 2(y² + z²))
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

} // namespace perseus_lite_hud
