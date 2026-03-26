#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace perseus_vision
{

    class StereoOdom : public rclcpp::Node
    {
    public:
        StereoOdom();

    private:
        using Image = sensor_msgs::msg::Image;
        using ImageSubscriber = message_filters::Subscriber<Image>;
        using ImageSyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
        using ImageSynchronizer = message_filters::Synchronizer<ImageSyncPolicy>;

        struct CameraModel
        {
            bool valid{false};
            double fx{0.0};
            double fy{0.0};
            double cx{0.0};
            double cy{0.0};
            double baseline_m{0.0};
        };

        struct FrameState
        {
            rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
            cv::Mat left_image;
            cv::Mat disparity;
            std::vector<cv::Point2f> features;
        };

        struct RigidTransform
        {
            bool valid{false};
            cv::Matx33d rotation = cv::Matx33d::eye();
            cv::Vec3d translation{0.0, 0.0, 0.0};
            std::vector<int> inlier_indices;
            double mean_error_m{0.0};
        };

        void left_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void right_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void stereo_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);

        bool update_camera_model_locked(const sensor_msgs::msg::CameraInfo& left_info, const sensor_msgs::msg::CameraInfo* right_info);
        cv::Mat prepare_grayscale_image(const sensor_msgs::msg::Image& msg) const;
        cv::Mat compute_disparity(const cv::Mat& left_image, const cv::Mat& right_image);
        std::vector<cv::Point2f> detect_features(const cv::Mat& image, const cv::Mat& disparity) const;
        bool reproject_to_3d(const cv::Point2f& pixel, const cv::Mat& disparity, cv::Vec3d& point_3d) const;
        RigidTransform estimate_motion_ransac(
            const std::vector<cv::Vec3d>& previous_points,
            const std::vector<cv::Vec3d>& current_points) const;
        bool estimate_rigid_transform(
            const std::vector<cv::Vec3d>& previous_points,
            const std::vector<cv::Vec3d>& current_points,
            const std::vector<int>& indices,
            cv::Matx33d& rotation,
            cv::Vec3d& translation,
            double& mean_error_m) const;
        void reset_tracking(const cv::Mat& left_image, const cv::Mat& disparity, const rclcpp::Time& stamp);
        void publish_odometry(const rclcpp::Time& stamp, const cv::Matx33d& delta_rotation, const cv::Vec3d& delta_translation, double dt_s);
        bool resolve_output_pose(
            cv::Matx33d& world_rotation_from_child,
            cv::Vec3d& world_translation_from_child,
            cv::Matx33d& child_rotation_from_tracking) const;

        std::string left_image_topic_;
        std::string right_image_topic_;
        std::string left_camera_info_topic_;
        std::string right_camera_info_topic_;
        std::string odom_topic_;
        std::string odom_frame_id_;
        std::string tracking_frame_id_;
        std::string child_frame_id_;

        bool publish_tf_{true};
        bool use_clahe_{true};
        bool use_equalize_hist_{false};
        int sync_queue_size_{10};
        int max_features_{250};
        int min_tracked_features_{80};
        double feature_quality_level_{0.01};
        double min_feature_distance_px_{12.0};
        int feature_block_size_{5};
        int lk_window_size_px_{21};
        int lk_max_pyramid_level_{3};
        int min_inliers_{20};
        int ransac_iterations_{120};
        double ransac_inlier_threshold_m_{0.10};
        double max_frame_delta_s_{0.25};
        double max_valid_depth_m_{6.0};
        double min_valid_depth_m_{0.2};
        double min_valid_disparity_px_{1.0};
        double max_translation_per_frame_m_{0.50};
        double max_rotation_per_frame_rad_{0.70};
        int stereo_num_disparities_{64};
        int stereo_block_size_{15};
        int stereo_pre_filter_cap_{31};
        int stereo_texture_threshold_{10};
        int stereo_uniqueness_ratio_{10};
        int stereo_speckle_window_size_{50};
        int stereo_speckle_range_{2};
        int stereo_disp12_max_diff_{1};

        mutable std::mutex camera_model_mutex_;
        CameraModel camera_model_;
        sensor_msgs::msg::CameraInfo::SharedPtr latest_left_camera_info_;
        sensor_msgs::msg::CameraInfo::SharedPtr latest_right_camera_info_;

        bool has_previous_frame_{false};
        FrameState previous_frame_;

        cv::Matx33d world_rotation_from_camera_ = cv::Matx33d::eye();
        cv::Vec3d world_translation_from_camera_{0.0, 0.0, 0.0};

        ImageSubscriber left_image_sub_;
        ImageSubscriber right_image_sub_;
        std::shared_ptr<ImageSynchronizer> stereo_sync_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        cv::Ptr<cv::CLAHE> clahe_;
        cv::Ptr<cv::StereoBM> stereo_matcher_;
    };

}  // namespace perseus_vision
