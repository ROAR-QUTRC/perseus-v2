#include "perseus_vision/object_map_server.hpp"

ObjectMapServer::ObjectMapServer()
    : Node("object_map_server")
{
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    tf_output_frame_ = this->declare_parameter<std::string>("tf_output_frame", "map");
    capture_images_ = this->declare_parameter<bool>("capture_images", false);
    aruco_detect_topic_ = this->declare_parameter<std::string>("aruco_detect_topic", "/detection/aruco");
    cube_detect_topic_ = this->declare_parameter<std::string>("cube_detect_topic", "/detection/cube");
    capture_aruco_ = this->declare_parameter<bool>("capture_aruco", true);
    capture_cube_ = this->declare_parameter<bool>("capture_cube", true);
    capture_radius_ = this->declare_parameter<int>("capture_radius", 5); // Meters (tolerance around detected object to capture image)
    targets_file_ = this->declare_parameter<std::string>("targets_file", "perseus-v2/software/ros_ws/src/perseus_vision/config/targets.yaml");
    
    // Initialize TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
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
                double yaw = item["yaw"].as<double>();
                // Convert yaw to quaternion (rotation around Z axis)
                double half_yaw = yaw / 2.0;
                t.qx = 0.0;
                t.qy = 0.0;
                t.qz = std::sin(half_yaw);
                t.qw = std::cos(half_yaw);
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
    
    for (const auto& [name, target] : targets_) {
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
  }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectMapServer>());
    rclcpp::shutdown();
    return 0;
}
