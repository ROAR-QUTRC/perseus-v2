#include "perseus_vision/object_map_server.hpp"


ObjectMapServer::ObjectMapServer()
    : Node("object_map_server")
{
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    tf_output_frame_ = this->declare_parameter<std::string>("tf_output_frame", "map");
    capture_images_ = this->declare_parameter<bool>("capture_images", false);
    aruco_detect_topic_ = this->declare_parameter<std::string>("aruco_detect_topic", "/detection/aruco");
    cube_detect_topic_ = this->declare_parameter<std::string>("cube_detect_topic", "/detection/cube");
    capture_aruco_ = this->declare_parameter<bool>("enable_aruco_tracking", true);
    capture_cube_ = this->declare_parameter<bool>("enable_cube_tracking", true);
    capture_radius_ = this->declare_parameter<int>("capture_radius", 3); // Meters (tolerance around detected object to capture image)
    max_detection_distance_ = this->declare_parameter<double>("max_detection_distance", 7.0); // Maximum distance from robot to ArUco marker for renaming
    targets_file_ = this->declare_parameter<std::string>("targets_file", "perseus-v2/software/ros_ws/src/perseus_vision/config/targets.yaml");
    robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame", "base_link");
    // Initialize TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize TF2 buffer and listener for transform lookups
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Load targets from YAML file
    load_targets_and_broadcast();
    
    // Set up timer to broadcast target frames at 10 Hz
    tf_broadcast_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObjectMapServer::broadcast_target_frames, this));
    
    // Image subscriber and publisher
    if (capture_aruco_) 
    {
        aruco_detections_sub_ =
            this->create_subscription<perseus_vision::msg::ObjectDetections>(
                aruco_detect_topic_, 10,
                std::bind(&ObjectMapServer::aruco_callback, this, std::placeholders::_1));
    }
    if (capture_cube_) 
    {
        cube_detections_sub_ =
            this->create_subscription<perseus_vision::msg::ObjectDetections>(
                cube_detect_topic_, 10,
                std::bind(&ObjectMapServer::cube_callback, this, std::placeholders::_1));
    }
    RCLCPP_INFO(this->get_logger(), "Perseus' ObjectMapServer node started.");
}

void ObjectMapServer::load_targets_and_broadcast()
{
    // Expand file path - if relative, prepend workspace root
    std::string expanded_path = targets_file_;
    
    // If path is relative (doesn't start with /), prepend $HOME and workspace path
    if (expanded_path[0] != '/') {
        const char* home = std::getenv("HOME");
        if (home) {
            expanded_path = std::string(home) + "/" + expanded_path;
        }
    }
    
    try {
        YAML::Node config = YAML::LoadFile(expanded_path);
        
        // Try both "waypoints" and "targets" sections for compatibility
        const YAML::Node& waypoints_node = config["waypoints"];
        const YAML::Node& targets_node = config["targets"];
        
        const YAML::Node& node_to_parse = waypoints_node ? waypoints_node : targets_node;
        
        if (!node_to_parse) {
            RCLCPP_ERROR(this->get_logger(), "No 'waypoints' or 'targets' section found in %s", expanded_path.c_str());
            return;
        }
        
        if (!node_to_parse.IsSequence()) {
            RCLCPP_ERROR(this->get_logger(), "waypoints/targets must be a sequence in %s", expanded_path.c_str());
            return;
        }
        
        for (const auto& item : node_to_parse) {
            std::string name = item["name"].as<std::string>();
            
            Target t;
            t.frame_id = name;
            t.x = item["x"].as<double>(0.0);
            t.y = item["y"].as<double>(0.0);
            t.z = item["z"].as<double>(0.0);
            
            // Check if yaw is provided (convert to quaternion)
            if (item["yaw"]) {
                const double yaw = item["yaw"].as<double>();

                // Build an ArUco-style frame in the MAP frame:
                //   Z_marker = outward normal in XY plane (from yaw)
                //   Y_marker = down (OpenCV convention)
                //   X_marker = Y x Z (right-handed)
                tf2::Vector3 z_m(std::cos(yaw), std::sin(yaw), 0.0);  // marker normal in map XY
                tf2::Vector3 y_m(0.0, 0.0, 1.0);                    // marker "down" in map
                tf2::Vector3 x_m = y_m.cross(z_m);                   // right-handed: X = Y x Z

                // Normalize just in case
                z_m.normalize();
                x_m.normalize();

                // Rotation matrix with columns = (X, Y, Z) axes expressed in parent(map)
                tf2::Matrix3x3 R(
                    x_m.x(), y_m.x(), z_m.x(),
                    x_m.y(), y_m.y(), z_m.y(),
                    x_m.z(), y_m.z(), z_m.z()
                );

                tf2::Quaternion q;
                R.getRotation(q);
                q.normalize();

                t.qx = q.x();
                t.qy = q.y();
                t.qz = q.z();
                t.qw = q.w();
            } else {
                // Use explicit quaternion if provided
                t.qx = item["qx"].as<double>(0.0);
                t.qy = item["qy"].as<double>(0.0);
                t.qz = item["qz"].as<double>(0.0);
                t.qw = item["qw"].as<double>(1.0);
            }

            targets_[name] = t;
            RCLCPP_INFO(this->get_logger(), "Loaded waypoint: %s at (%.2f, %.2f, %.2f) yaw=%.3f rad", 
                       name.c_str(), t.x, t.y, t.z, 
                       2.0 * std::atan2(t.qz, t.qw));
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints from %s", 
                   targets_.size(), expanded_path.c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load targets file %s: %s", 
                    expanded_path.c_str(), e.what());
    }
}

void ObjectMapServer::broadcast_target_frames()
{
    auto now = this->get_clock()->now();
    
    for (const auto& [name, target] : targets_) { // Scan all targets and broadcast
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = tf_output_frame_;
        t.child_frame_id = target.frame_id;
        
        t.transform.translation.x = target.x;
        t.transform.translation.y = target.y;
        t.transform.translation.z = target.z;
        
        t.transform.rotation.x = target.qx;
        t.transform.rotation.y = target.qy;
        t.transform.rotation.z = target.qz;
        t.transform.rotation.w = target.qw;
        
        tf_broadcaster_->sendTransform(t);
    }
}

void ObjectMapServer::aruco_callback(
  const perseus_vision::msg::ObjectDetections::SharedPtr msg)
{
  // Time
  rclcpp::Time t(msg->stamp);

  // Frame
  const std::string & frame = msg->frame_id;

  // Sanity check
  if (msg->ids.size() != msg->poses.size()) {
    RCLCPP_WARN(this->get_logger(),
      "ids.size() != poses.size() (%zu vs %zu)",
      msg->ids.size(), msg->poses.size());
    return;
  }

  // Iterate detections
  for (size_t i = 0; i < msg->ids.size(); ++i) {
    int id = msg->ids[i];
    const geometry_msgs::msg::Pose & pose = msg->poses[i];

    RCLCPP_INFO(this->get_logger(),
      "Aruco ID=%d in frame=%s at t=%.3f",
      id, frame.c_str(), t.seconds());

    // Transform marker pose from map frame to base_link frame to get distance from robot
    double aruco_distance_from_robot = std::numeric_limits<double>::max();
    try {
      geometry_msgs::msg::PoseStamped marker_pose_map;
      marker_pose_map.header.frame_id = frame;
      marker_pose_map.header.stamp = msg->stamp;
      marker_pose_map.pose = pose;

      // Transform marker pose from map/odom frame to base_link frame
      geometry_msgs::msg::PoseStamped marker_pose_baselink;
      tf_buffer_->transform(marker_pose_map, marker_pose_baselink, robot_base_frame_, tf2::Duration(0));

      // Calculate distance from base_link origin to marker
      aruco_distance_from_robot = std::sqrt(
        marker_pose_baselink.pose.position.x * marker_pose_baselink.pose.position.x + 
        marker_pose_baselink.pose.position.y * marker_pose_baselink.pose.position.y + 
        marker_pose_baselink.pose.position.z * marker_pose_baselink.pose.position.z);
    }
    catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(),
        "Could not transform marker pose to base_link: %s", ex.what());
      continue;
    }
    
    if (aruco_distance_from_robot > max_detection_distance_) {
      RCLCPP_DEBUG(this->get_logger(),
        "ArUco marker %d too far (%.2f m > max %.2f m). Skipping target matching.",
        id, aruco_distance_from_robot, max_detection_distance_);
      continue;
    }

    // Check if this ArUco is within capture_radius of any unlocked target
    double aruco_x = pose.position.x;
    double aruco_y = pose.position.y;
    std::string target_to_rename;
    double closest_distance = capture_radius_;

    for (const auto& [target_name, target] : targets_) { // Scan all targets
      // Skip locked targets - they cannot be renamed
      if (target.locked) {
        continue;
      }
      
      double dx = aruco_x - target.x;
      double dy = aruco_y - target.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < closest_distance) {
        closest_distance = distance;
        target_to_rename = target_name;
      }
    }

    // Rename the closest target if it's within capture radius
    if (!target_to_rename.empty()) {
      std::string new_name = "aruco_" + std::to_string(id);
      RCLCPP_INFO(this->get_logger(),
        "ArUco marker %d found within capture radius (%.2f m) of target '%s'. Renaming to '%s' (LOCKED)",
        id, closest_distance, target_to_rename.c_str(), new_name.c_str());

      // Create updated target with new name
      Target target_copy = targets_[target_to_rename];
      target_copy.frame_id = new_name;
      target_copy.locked = true;  // Lock the target so it cannot be renamed again
      
      // Remove old entry and add new one (permanently stores the rename)
      targets_.erase(target_to_rename);
      targets_[new_name] = target_copy;
    }
  }
}


void ObjectMapServer::cube_callback(
  const perseus_vision::msg::ObjectDetections::SharedPtr msg)
{
  // Currently not implemented
  RCLCPP_INFO(this->get_logger(),
    "Cube detection callback received %zu detections (not implemented).",
    msg->ids.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectMapServer>());
    rclcpp::shutdown();
    return 0;
}
