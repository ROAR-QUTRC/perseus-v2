#ifndef CUBE_DETECTOR_HPP
#define CUBE_DETECTOR_HPP

#include <onnxruntime_cxx_api.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
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

struct Detection
{
    cv::Rect box;
    int cls;
    float score;
};

inline float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

inline float IoU(const cv::Rect& a, const cv::Rect& b)
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

inline std::vector<Detection> nms(const std::vector<Detection>& dets, float iou_the)
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

class CubeDetector : public rclcpp::Node
{
public:
    CubeDetector();

private:
    double calculateDistance(const geometry_msgs::msg::PoseStamped& pose1,
                             const geometry_msgs::msg::PoseStamped& pose2);

    int findClosestCube(const geometry_msgs::msg::PoseStamped& new_pose);

    void updateOrCreateCube(const geometry_msgs::msg::PoseStamped& world_pose);

    void cleanupOldCubes();

    void depthCb(const sensor_msgs::msg::Image::SharedPtr msg);

    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg);

    const char** input_names_cstr();

    const char** output_names_cstr();

    std::string getClassName(int class_id);

    // Publishers and Subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    // Depth image storage
    cv::Mat latest_depth_image_;
    std::mutex depth_mutex_;

    // Camera parameters
    double camera_fx_, camera_fy_, camera_cx_, camera_cy_;
    std::string camera_frame_;
    bool publish_tf_;
    std::string tf_output_frame_;
    bool compressed_io_;
    std::string model_;
    std::string input_img_;
    std::string input_depth_;
    bool publish_img_;
    std::string output_img_;
    bool publish_output_;
    std::string output_topic_;
    bool use_cuda_;

    // Cube tracking
    std::unordered_map<int, TrackedCube> tracked_cubes_;
    std::mutex cubes_mutex_;
    int next_cube_id_;
    double cube_distance_threshold_;  // meters
    int min_detections_for_stable_;   // number of detections needed

    // TF2 components
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ONNX Runtime components
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::MemoryInfo> memory_info_;
    std::vector<std::string> input_names_, output_names_;
    std::vector<const char*> input_cstr_, output_cstr_;
    bool model_loaded_ = false;

    // Cached detection parameters (optimization)
    float confidence_threshold_;
    float nms_threshold_;
    std::vector<Detection> detections_cache_;  // Pre-allocated for reuse
    std::vector<float> chw_cache_;  // Pre-allocated CHW buffer
};

#endif  // CUBE_DETECTOR_HPP
