#include "perseus_vision/cube_detector.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

CubeDetector::CubeDetector()
    : Node("cube_detector")
{
    // Declare parameters
    this->declare_parameter("confidence_threshold", 0.2);
    this->declare_parameter("nms_threshold", 0.5);
    this->declare_parameter("cube_distance_threshold", 0.5);
    this->declare_parameter("min_detections_for_stable", 5);
    this->declare_parameter(
        "camera_matrix",
        std::vector<double>{530.4, 0.0, 320.0,
                            0.0, 530.4, 240.0,
                            0.0, 0.0, 1.0});
    this->declare_parameter("camera_frame", "camera_link_optical");
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("tf_output_frame", "odom");
    this->declare_parameter("compressed_io", false);
    this->declare_parameter("input_img", "/rgbd_camera/image_raw");
    this->declare_parameter("publish_img", true);
    this->declare_parameter("output_img", "/detection/cube/image_raw");
    this->declare_parameter("publish_output", true);
    this->declare_parameter("output_topic", "/detection/cube");
    this->declare_parameter("model_path", "/model/best.onnx");
    // Get parameters
    auto camera_matrix = this->get_parameter("camera_matrix").as_double_array();
    camera_fx_ = camera_matrix[0];
    camera_fy_ = camera_matrix[4];
    camera_cx_ = camera_matrix[2];
    camera_cy_ = camera_matrix[5];
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    tf_output_frame_ = this->get_parameter("tf_output_frame").as_string();
    compressed_io_ = this->get_parameter("compressed_io").as_bool();
    input_img_ = this->get_parameter("input_img").as_string();
    publish_img_ = this->get_parameter("publish_img").as_bool();
    output_img_ = this->get_parameter("output_img").as_string();
    publish_output_ = this->get_parameter("publish_output").as_bool();
    output_topic_ = this->get_parameter("output_topic").as_string();
    model_ = this->get_parameter("model_path").as_string();
    (void)publish_output_;  // currently unused, but kept for future use

    // Initialize cube tracking
    next_cube_id_ = 0;
    cube_distance_threshold_ =
        this->get_parameter("cube_distance_threshold").as_double();
    min_detections_for_stable_ =
        static_cast<int>(this->get_parameter("min_detections_for_stable").as_int());

    // Create publishers
    if (publish_img_)
    {
        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(output_img_, 10);
    }
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);

    // Create subscriptions
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_img_, 10,
        std::bind(&CubeDetector::imageCb, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/rgbd_camera/depth/image_raw", 10,
        std::bind(&CubeDetector::depthCb, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::string model_path =
        ament_index_cpp::get_package_share_directory("perseus_vision") +
        model_;
    RCLCPP_INFO(get_logger(), "Loading model: %s", model_path.c_str());

    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "yolo");
    Ort::SessionOptions so;
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), so);
    memory_info_ = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    Ort::AllocatorWithDefaultOptions allocator;
    size_t in_count = session_->GetInputCount();
    input_names_.resize(in_count);
    for (size_t i = 0; i < in_count; ++i)
    {
        input_names_[i] = session_->GetInputNameAllocated(i, allocator).get();
    }

    size_t out_count = session_->GetOutputCount();
    output_names_.resize(out_count);
    for (size_t i = 0; i < out_count; ++i)
    {
        output_names_[i] = session_->GetOutputNameAllocated(i, allocator).get();
    }

    model_loaded_ = true;
}

double CubeDetector::calculateDistance(
    const geometry_msgs::msg::PoseStamped &pose1,
    const geometry_msgs::msg::PoseStamped &pose2)
{
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    double dz = pose1.pose.position.z - pose2.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int CubeDetector::findClosestCube(const geometry_msgs::msg::PoseStamped &new_pose)
{
    // NOTE: Caller must hold cubes_mutex_ before calling this function.
    double min_distance = std::numeric_limits<double>::max();
    int closest_id = -1;

    for (const auto &[id, cube] : tracked_cubes_)
    {
        double distance = calculateDistance(new_pose, cube.world_pose);
        if (distance < min_distance && distance < cube_distance_threshold_)
        {
            min_distance = distance;
            closest_id = id;
        }
    }
    return closest_id;
}

void CubeDetector::updateOrCreateCube(
    const geometry_msgs::msg::PoseStamped &world_pose)
{
    std::lock_guard<std::mutex> lock(cubes_mutex_);

    // Find the closest existing cube while holding the mutex
    int cube_id = findClosestCube(world_pose);

    if (cube_id == -1)
    {
        // Create new cube
        cube_id = next_cube_id_++;

        TrackedCube new_cube;
        new_cube.world_pose = world_pose;
        new_cube.last_seen = this->get_clock()->now();
        new_cube.detection_count = 1;
        new_cube.is_stable = false;

        tracked_cubes_[cube_id] = new_cube;

        RCLCPP_INFO(
            get_logger(),
            "New cube %d created at [%.3f, %.3f, %.3f]",
            cube_id,
            world_pose.pose.position.x,
            world_pose.pose.position.y,
            world_pose.pose.position.z);
    }
    else
    {
        // Update existing cube
        TrackedCube &cube = tracked_cubes_[cube_id];

        if (!cube.is_stable)
        {
            // Still learning the cube's position, so update it
            cube.detection_count++;

            // Use exponential moving average to stabilize position
            double alpha = 0.3; // weight for new observation
            cube.world_pose.pose.position.x =
                alpha * world_pose.pose.position.x +
                (1.0 - alpha) * cube.world_pose.pose.position.x;
            cube.world_pose.pose.position.y =
                alpha * world_pose.pose.position.y +
                (1.0 - alpha) * cube.world_pose.pose.position.y;
            cube.world_pose.pose.position.z =
                alpha * world_pose.pose.position.z +
                (1.0 - alpha) * cube.world_pose.pose.position.z;

            if (cube.detection_count >= min_detections_for_stable_)
            {
                cube.is_stable = true;
                RCLCPP_INFO(
                    get_logger(),
                    "Cube %d is now stable at [%.3f, %.3f, %.3f]",
                    cube_id,
                    cube.world_pose.pose.position.x,
                    cube.world_pose.pose.position.y,
                    cube.world_pose.pose.position.z);
            }
        }

        cube.last_seen = this->get_clock()->now();
        cube.world_pose.header = world_pose.header; // Update timestamp and frame
    }

    // Broadcast TF for this cube using configured output frame
    const TrackedCube &cube = tracked_cubes_[cube_id];

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = tf_output_frame_;       // use param, not hard-coded "odom"
    tf_msg.child_frame_id = "cube_" + std::to_string(cube_id);
    tf_msg.transform.translation.x = cube.world_pose.pose.position.x;
    tf_msg.transform.translation.y = cube.world_pose.pose.position.y;
    tf_msg.transform.translation.z = cube.world_pose.pose.position.z;
    tf_msg.transform.rotation = cube.world_pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
}

void CubeDetector::cleanupOldCubes()
{
    std::lock_guard<std::mutex> lock(cubes_mutex_);
    auto current_time = this->get_clock()->now();
    auto max_age = rclcpp::Duration::from_seconds(10.0); // 10 seconds timeout

    auto it = tracked_cubes_.begin();
    while (it != tracked_cubes_.end())
    {
        if (current_time - it->second.last_seen > max_age)
        {
            RCLCPP_INFO(get_logger(), "Removing old cube %d", it->first);
            it = tracked_cubes_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void CubeDetector::depthCb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat depth;
        if (msg->encoding == "16UC1") // Depth in millimeters
        {
            depth = cv_bridge::toCvShare(
                        msg, sensor_msgs::image_encodings::TYPE_16UC1)
                        ->image.clone();
        }
        else if (msg->encoding == "32FC1") // Depth in meters
        {
            depth = cv_bridge::toCvShare(
                        msg, sensor_msgs::image_encodings::TYPE_32FC1)
                        ->image.clone();
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(), "Unsupported depth encoding: %s",
                msg->encoding.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(depth_mutex_);
        latest_depth_image_ = depth;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            get_logger(), "Failed to convert depth image: %s", e.what());
    }
}

void CubeDetector::imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!model_loaded_)
    {
        return;
    }

    cv::Mat bgr;
    try
    {
        bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone(); // Clone to ensure we have our own copy
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "Failed to convert image.");
        return;
    }

    int ow = bgr.cols, oh = bgr.rows;

    // Prepare input for ONNX model
    cv::Mat input;
    cv::cvtColor(bgr, input, cv::COLOR_BGR2RGB);
    cv::resize(input, input, cv::Size(640, 640));
    input.convertTo(input, CV_32F, 1.0 / 255.0);

    std::vector<float> chw(3 * 640 * 640);
    for (int c = 0; c < 3; ++c)
    {
        for (int y = 0; y < 640; ++y)
        {
            for (int x = 0; x < 640; ++x)
            {
                chw[c * 640 * 640 + y * 640 + x] =
                    input.at<cv::Vec3f>(y, x)[c];
            }
        }
    }

    std::array<int64_t, 4> shape{1, 3, 640, 640};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        *memory_info_, chw.data(), chw.size(), shape.data(), shape.size());

    auto output = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_cstr(), &input_tensor, 1,
        output_names_cstr(), 1);

    const float *out = output[0].GetTensorData<float>();
    auto dims =
        output[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_feats = dims[1];
    int num_preds = dims[2];

    float scale_x = static_cast<float>(ow) / 640.0f;
    float scale_y = static_cast<float>(oh) / 640.0f;
    float conf_threshold = static_cast<float>(
        this->get_parameter("confidence_threshold").as_double());
    float nms_threshold = static_cast<float>(
        this->get_parameter("nms_threshold").as_double());

    std::vector<Detection> detections;
    for (int i = 0; i < num_preds; ++i)
    {
        float xc = out[0 * num_preds + i];
        float yc = out[1 * num_preds + i];
        float w = out[2 * num_preds + i];
        float h = out[3 * num_preds + i];
        float obj = sigmoid(out[4 * num_preds + i]);

        int cls = -1;
        float best = -1e9f;
        for (int j = 5; j < num_feats; ++j)
        {
            float val = out[j * num_preds + i];
            if (val > best)
            {
                best = val;
                cls = j - 5;
            }
        }

        float conf = obj * sigmoid(best);
        if (conf < conf_threshold)
        {
            continue;
        }

        int x1 = static_cast<int>((xc - w / 2) * scale_x);
        int y1 = static_cast<int>((yc - h / 2) * scale_y);
        int x2 = static_cast<int>((xc + w / 2) * scale_x);
        int y2 = static_cast<int>((yc + h / 2) * scale_y);
        detections.push_back(
            {cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), cls, conf});
    }

    auto final_dets = nms(detections, nms_threshold);

    // Process each detected cube
    for (const auto &d : final_dets)
    {
        cv::Point center(
            d.box.x + d.box.width / 2,
            d.box.y + d.box.height / 2);

        // Get actual depth from depth image
        float Z = 1.5f; // fallback depth in meters
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            if (!latest_depth_image_.empty() &&
                center.x >= 0 &&
                center.x < latest_depth_image_.cols &&
                center.y >= 0 &&
                center.y < latest_depth_image_.rows)
            {
                if (latest_depth_image_.type() == CV_16UC1)
                {
                    // Depth in millimeters, convert to meters
                    uint16_t depth_mm =
                        latest_depth_image_.at<uint16_t>(center.y, center.x);
                    if (depth_mm > 0)
                    {
                        Z = static_cast<float>(depth_mm) / 1000.0f;
                    }
                }
                else if (latest_depth_image_.type() == CV_32FC1)
                {
                    // Depth already in meters
                    float depth_m =
                        latest_depth_image_.at<float>(center.y, center.x);
                    if (depth_m > 0.1f && depth_m < 10.0f)
                    { // reasonable depth range
                        Z = depth_m;
                    }
                }
            }
        }

        // Camera intrinsics from cached parameters
        float fx = camera_fx_;
        float fy = camera_fy_;
        float cx = camera_cx_;
        float cy = camera_cy_;

        // Convert pixel coordinates to 3D coordinates in camera optical frame
        // Using OpenCV coordinate system first
        float X_cv = (center.x - cx) * Z / fx; // Right in image
        float Y_cv = (center.y - cy) * Z / fy; // Down in image
        float Z_cv = Z;                        // Forward from camera

        geometry_msgs::msg::PoseStamped cam_pose;
        cam_pose.header = msg->header;
        cam_pose.header.frame_id = camera_frame_;

        // Apply same OpenCV to ROS coordinate transformation as ArUco detector
        cam_pose.pose.position.x = Z_cv;   // Z becomes X (forward -> X)
        cam_pose.pose.position.y = -X_cv;  // -X becomes Y (right -> -Y)
        cam_pose.pose.position.z = -Y_cv;  // -Y becomes Z (down -> -Z)
        cam_pose.pose.orientation.w = 1.0; // identity quaternion

        if (publish_tf_)
        {
            try
            {
                // Transform pose to output frame
                geometry_msgs::msg::PoseStamped odom_pose =
                    tf_buffer_->transform(
                        cam_pose,
                        tf_output_frame_,
                        tf2::durationFromSec(0.1));

                // Update or create tracked cube (this handles the stable positioning)
                updateOrCreateCube(odom_pose);

                // Still publish the raw detection for debugging
                pub_pose_->publish(odom_pose);

                RCLCPP_DEBUG(
                    get_logger(),
                    "Detection: pixel[%d,%d] depth=%.3fm "
                    "cv[%.3f,%.3f,%.3f] "
                    "ros[%.3f,%.3f,%.3f] "
                    "odom[%.3f,%.3f,%.3f]",
                    center.x, center.y, Z,
                    X_cv, Y_cv, Z_cv,
                    cam_pose.pose.position.x,
                    cam_pose.pose.position.y,
                    cam_pose.pose.position.z,
                    odom_pose.pose.position.x,
                    odom_pose.pose.position.y,
                    odom_pose.pose.position.z);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(get_logger(), "TF2 transform failed: %s", ex.what());
            }
        }

        // Draw rectangle on image for visualization
        cv::rectangle(bgr, d.box, cv::Scalar(0, 255, 0), 2);
    }

    // Periodically clean up old cubes (every 30 frames approximately)
    static int frame_count = 0;
    if (++frame_count % 30 == 0)
    {
        cleanupOldCubes();
    }

    if (pub_image_)
    {
        auto out_msg =
            cv_bridge::CvImage(msg->header, "bgr8", bgr).toImageMsg();
        pub_image_->publish(*out_msg);
    }
}

const char **CubeDetector::input_names_cstr()
{
    input_cstr_.clear();
    for (auto &s : input_names_)
    {
        input_cstr_.push_back(s.c_str());
    }
    return input_cstr_.data();
}

const char **CubeDetector::output_names_cstr()
{
    output_cstr_.clear();
    for (auto &s : output_names_)
    {
        output_cstr_.push_back(s.c_str());
    }
    return output_cstr_.data();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeDetector>());
    rclcpp::shutdown();
    return 0;
}
