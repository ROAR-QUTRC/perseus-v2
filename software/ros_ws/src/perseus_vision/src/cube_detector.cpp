#include <onnxruntime_cxx_api.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>
#include <vector>

namespace
{
    struct Detection
    {
        cv::Rect box;
        int cls;
        float score;
    };

    struct TrackedCube
    {
        geometry_msgs::msg::PoseStamped world_pose;
        rclcpp::Time last_seen;
        int detection_count;
        bool is_stable;

        TrackedCube()
            : detection_count(0),
              is_stable(false)
        {
        }
    };

    float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

    float IoU(const cv::Rect& a, const cv::Rect& b)
    {
        int x1 = std::max(a.x, b.x);
        int y1 = std::max(a.y, b.y);
        int x2 = std::min(a.x + a.width, b.x + b.width);
        int y2 = std::min(a.y + a.height, b.y + b.height);
        int inter = std::max(0, x2 - x1) * std::max(0, y2 - y1);
        int areaA = a.width * a.height;
        int areaB = b.width * b.height;
        int uni = areaA + areaB - inter;
        return uni > 0 ? static_cast<float>(inter) / static_cast<float>(uni) : 0.f;
    }

    std::vector<Detection> nms(const std::vector<Detection>& dets, float iou_the)
    {
        std::vector<Detection> result;
        std::map<int, std::vector<Detection>> buckets;
        for (auto& d : dets) buckets[d.cls].push_back(d);

        for (auto& [cls, vec] : buckets)
        {
            std::sort(vec.begin(), vec.end(), [](const Detection& a, const Detection& b)
                      { return a.score > b.score; });
            std::vector<bool> removed(vec.size(), false);
            for (size_t i = 0; i < vec.size(); ++i)
            {
                if (removed[i])
                    continue;
                result.push_back(vec[i]);
                for (size_t j = i + 1; j < vec.size(); ++j)
                {
                    if (!removed[j] && IoU(vec[i].box, vec[j].box) > iou_the)
                        removed[j] = true;
                }
            }
        }
        return result;
    }
}

class CubeDetector : public rclcpp::Node
{
public:
    CubeDetector()
        : Node("cube_detector")
    {
        // Declare parameters for camera intrinsics (matching vision_params.yaml)
        this->declare_parameter("camera.fx", 530.4);
        this->declare_parameter("camera.fy", 530.4);
        this->declare_parameter("camera.cx", 320.0);
        this->declare_parameter("camera.cy", 240.0);
        this->declare_parameter("confidence_threshold", 0.3);
        this->declare_parameter("nms_threshold", 0.5);
        this->declare_parameter("cube_distance_threshold", 0.5);  // meters
        this->declare_parameter("min_detections_for_stable", 5);

        // Initialize cube tracking
        next_cube_id_ = 0;
        cube_distance_threshold_ = this->get_parameter("cube_distance_threshold").as_double();
        min_detections_for_stable_ = static_cast<int>(this->get_parameter("min_detections_for_stable").as_int());

        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/rgbd_camera/image/cube_detection", 10);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cube_detector/cube_pose", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgbd_camera/image_raw", 10,
            std::bind(&CubeDetector::imageCb, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgbd_camera/depth/image_raw", 10,
            std::bind(&CubeDetector::depthCb, this, std::placeholders::_1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::string model_path = ament_index_cpp::get_package_share_directory("perseus_vision") + "/yolo_model/best.onnx";
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
            input_names_[i] = session_->GetInputNameAllocated(i, allocator).get();

        size_t out_count = session_->GetOutputCount();
        output_names_.resize(out_count);
        for (size_t i = 0; i < out_count; ++i)
            output_names_[i] = session_->GetOutputNameAllocated(i, allocator).get();

        model_loaded_ = true;
    }

private:
    double calculateDistance(const geometry_msgs::msg::PoseStamped& pose1,
                             const geometry_msgs::msg::PoseStamped& pose2)
    {
        double dx = pose1.pose.position.x - pose2.pose.position.x;
        double dy = pose1.pose.position.y - pose2.pose.position.y;
        double dz = pose1.pose.position.z - pose2.pose.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    int findClosestCube(const geometry_msgs::msg::PoseStamped& new_pose)
    {
        std::lock_guard<std::mutex> lock(cubes_mutex_);
        double min_distance = std::numeric_limits<double>::max();
        int closest_id = -1;

        for (const auto& [id, cube] : tracked_cubes_)
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

    void updateOrCreateCube(const geometry_msgs::msg::PoseStamped& world_pose)
    {
        int cube_id = findClosestCube(world_pose);

        std::lock_guard<std::mutex> lock(cubes_mutex_);
        if (cube_id == -1)
        {
            // Create new cube
            cube_id = next_cube_id_++;
            tracked_cubes_[cube_id] = TrackedCube();
            tracked_cubes_[cube_id].world_pose = world_pose;
            tracked_cubes_[cube_id].last_seen = this->get_clock()->now();
            tracked_cubes_[cube_id].detection_count = 1;

            RCLCPP_INFO(get_logger(), "New cube %d created at [%.3f, %.3f, %.3f]",
                        cube_id, world_pose.pose.position.x,
                        world_pose.pose.position.y, world_pose.pose.position.z);
        }
        else
        {
            // Update existing cube
            TrackedCube& cube = tracked_cubes_[cube_id];

            if (!cube.is_stable)
            {
                // Still learning the cube's position, so update it
                cube.detection_count++;

                // Use exponential moving average to stabilize position
                double alpha = 0.3;  // weight for new observation
                cube.world_pose.pose.position.x = alpha * world_pose.pose.position.x +
                                                  (1 - alpha) * cube.world_pose.pose.position.x;
                cube.world_pose.pose.position.y = alpha * world_pose.pose.position.y +
                                                  (1 - alpha) * cube.world_pose.pose.position.y;
                cube.world_pose.pose.position.z = alpha * world_pose.pose.position.z +
                                                  (1 - alpha) * cube.world_pose.pose.position.z;

                if (cube.detection_count >= min_detections_for_stable_)
                {
                    cube.is_stable = true;
                    RCLCPP_INFO(get_logger(), "Cube %d is now stable at [%.3f, %.3f, %.3f]",
                                cube_id, cube.world_pose.pose.position.x,
                                cube.world_pose.pose.position.y, cube.world_pose.pose.position.z);
                }
            }

            cube.last_seen = this->get_clock()->now();
            cube.world_pose.header = world_pose.header;  // Update timestamp and frame
        }

        // Broadcast stable TF for this cube
        const TrackedCube& cube = tracked_cubes_[cube_id];
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "cube_" + std::to_string(cube_id);
        tf_msg.transform.translation.x = cube.world_pose.pose.position.x;
        tf_msg.transform.translation.y = cube.world_pose.pose.position.y;
        tf_msg.transform.translation.z = cube.world_pose.pose.position.z;
        tf_msg.transform.rotation = cube.world_pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf_msg);
    }

    void cleanupOldCubes()
    {
        std::lock_guard<std::mutex> lock(cubes_mutex_);
        auto current_time = this->get_clock()->now();
        auto max_age = rclcpp::Duration::from_seconds(10.0);  // 10 seconds timeout

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

    void depthCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat depth;
            if (msg->encoding == "16UC1")
            {
                depth = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image.clone();
            }
            else if (msg->encoding == "32FC1")
            {
                depth = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone();
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Unsupported depth encoding: %s", msg->encoding.c_str());
                return;
            }

            std::lock_guard<std::mutex> lock(depth_mutex_);
            latest_depth_image_ = depth;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to convert depth image: %s", e.what());
        }
    }

    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!model_loaded_)
            return;

        cv::Mat bgr;
        try
        {
            bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
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
            for (int y = 0; y < 640; ++y)
                for (int x = 0; x < 640; ++x)
                    chw[c * 640 * 640 + y * 640 + x] = input.at<cv::Vec3f>(y, x)[c];

        std::array<int64_t, 4> shape{1, 3, 640, 640};
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(*memory_info_, chw.data(), chw.size(), shape.data(), shape.size());

        auto output = session_->Run(Ort::RunOptions{nullptr},
                                    input_names_cstr(), &input_tensor, 1,
                                    output_names_cstr(), 1);

        const float* out = output[0].GetTensorData<float>();
        auto dims = output[0].GetTensorTypeAndShapeInfo().GetShape();
        int num_feats = dims[1];
        int num_preds = dims[2];

        float scale_x = static_cast<float>(ow) / 640.0f;
        float scale_y = static_cast<float>(oh) / 640.0f;
        float conf_threshold = static_cast<float>(this->get_parameter("confidence_threshold").as_double());
        float nms_threshold = static_cast<float>(this->get_parameter("nms_threshold").as_double());

        std::vector<Detection> detections;
        for (int i = 0; i < num_preds; ++i)
        {
            float xc = out[0 * num_preds + i];
            float yc = out[1 * num_preds + i];
            float w = out[2 * num_preds + i];
            float h = out[3 * num_preds + i];
            float obj = sigmoid(out[4 * num_preds + i]);

            int cls = -1;
            float best = -1e9;
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
                continue;

            int x1 = static_cast<int>((xc - w / 2) * scale_x);
            int y1 = static_cast<int>((yc - h / 2) * scale_y);
            int x2 = static_cast<int>((xc + w / 2) * scale_x);
            int y2 = static_cast<int>((yc + h / 2) * scale_y);
            detections.push_back({cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), cls, conf});
        }

        auto final_dets = nms(detections, nms_threshold);

        // Process each detected cube
        for (const auto& d : final_dets)
        {
            cv::Point center(d.box.x + d.box.width / 2, d.box.y + d.box.height / 2);

            // Get actual depth from depth image
            float Z = 1.5f;  // fallback depth in meters
            {
                std::lock_guard<std::mutex> lock(depth_mutex_);
                if (!latest_depth_image_.empty() &&
                    center.x >= 0 && center.x < latest_depth_image_.cols &&
                    center.y >= 0 && center.y < latest_depth_image_.rows)
                {
                    if (latest_depth_image_.type() == CV_16UC1)
                    {
                        // Depth in millimeters, convert to meters
                        uint16_t depth_mm = latest_depth_image_.at<uint16_t>(center.y, center.x);
                        if (depth_mm > 0)
                        {
                            Z = static_cast<float>(depth_mm) / 1000.0f;
                        }
                    }
                    else if (latest_depth_image_.type() == CV_32FC1)
                    {
                        // Depth already in meters
                        float depth_m = latest_depth_image_.at<float>(center.y, center.x);
                        if (depth_m > 0.1f && depth_m < 10.0f)
                        {  // reasonable depth range
                            Z = depth_m;
                        }
                    }
                }
            }

            // Camera intrinsics from parameters
            float fx = static_cast<float>(this->get_parameter("camera.fx").as_double());
            float fy = static_cast<float>(this->get_parameter("camera.fy").as_double());
            float cx = static_cast<float>(this->get_parameter("camera.cx").as_double());
            float cy = static_cast<float>(this->get_parameter("camera.cy").as_double());

            // Convert pixel coordinates to 3D coordinates in camera optical frame
            // Using OpenCV coordinate system first
            float X_cv = (center.x - cx) * Z / fx;  // Right in image
            float Y_cv = (center.y - cy) * Z / fy;  // Down in image
            float Z_cv = Z;                         // Forward from camera

            geometry_msgs::msg::PoseStamped cam_pose;
            cam_pose.header = msg->header;
            cam_pose.header.frame_id = "camera_link_optical";

            // Apply same OpenCV to ROS coordinate transformation as ArUco detector
            cam_pose.pose.position.x = Z_cv;    // Z becomes X (forward becomes right)
            cam_pose.pose.position.y = -X_cv;   // -X becomes Y (right becomes forward)
            cam_pose.pose.position.z = -Y_cv;   // -Y becomes Z (down becomes up)
            cam_pose.pose.orientation.w = 1.0;  // identity quaternion

            try
            {
                // Transform pose to "odom" frame
                geometry_msgs::msg::PoseStamped odom_pose = tf_buffer_->transform(cam_pose, "odom", tf2::durationFromSec(0.1));

                // Update or create tracked cube (this handles the stable positioning)
                updateOrCreateCube(odom_pose);

                // Still publish the raw detection for debugging
                pub_pose_->publish(odom_pose);

                RCLCPP_DEBUG(get_logger(), "Detection: pixel[%d,%d] depth=%.3fm cv[%.3f,%.3f,%.3f] ros[%.3f,%.3f,%.3f] odom[%.3f,%.3f,%.3f]",
                             center.x, center.y, Z, X_cv, Y_cv, Z_cv,
                             cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z,
                             odom_pose.pose.position.x, odom_pose.pose.position.y, odom_pose.pose.position.z);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(get_logger(), "TF2 transform failed: %s", ex.what());
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

        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", bgr).toImageMsg();
        pub_image_->publish(*out_msg);
    }

    const char** input_names_cstr()
    {
        input_cstr_.clear();
        for (auto& s : input_names_) input_cstr_.push_back(s.c_str());
        return input_cstr_.data();
    }

    const char** output_names_cstr()
    {
        output_cstr_.clear();
        for (auto& s : output_names_) output_cstr_.push_back(s.c_str());
        return output_cstr_.data();
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    cv::Mat latest_depth_image_;
    std::mutex depth_mutex_;

    // Cube tracking
    std::unordered_map<int, TrackedCube> tracked_cubes_;
    std::mutex cubes_mutex_;
    int next_cube_id_;
    double cube_distance_threshold_;  // meters
    int min_detections_for_stable_;   // number of detections needed

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::MemoryInfo> memory_info_;
    std::vector<std::string> input_names_, output_names_;
    std::vector<const char*> input_cstr_, output_cstr_;
    bool model_loaded_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeDetector>());
    rclcpp::shutdown();
    return 0;
}
