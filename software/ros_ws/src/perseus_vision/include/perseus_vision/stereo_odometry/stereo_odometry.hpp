#pragma once

/// @file stereo_odometry.hpp
/// @brief Stereo visual odometry ROS 2 node built on libviso2.

#include <libviso2/viso_stereo.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>

#include "perseus_interfaces/msg/stereo_odometry_info.hpp"
#include "perseus_vision/stereo_odometry/odometry_publisher.hpp"

namespace perseus_vision
{
    /// @brief ROS 2 node computing stereo visual odometry with libviso2.
    ///
    /// Subscribes to a time-synchronized rectified stereo image + camera_info quartet,
    /// feeds libviso2's stereo feature matcher and motion estimator, and publishes the
    /// resulting incremental motion as odometry, a pose, TF, a sparse colour point cloud
    /// of inlier feature matches, and diagnostic counters. Pose integration and
    /// publishing is delegated to a composed OdometryPublisher.
    class StereoOdometry : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up the
        ///        synchronized subscribers, publishers, and odometry publisher.
        /// @param options Node options, supplied by the component container or by main().
        explicit StereoOdometry(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image,
            sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>;
        using ApproximateSynchronizer = message_filters::Synchronizer<ApproximateSyncPolicy>;

        /// @brief `ref_frame_change_method`: always advance the matching reference frame.
        static constexpr int32_t REF_FRAME_CHANGE_ALWAYS = 0;
        /// @brief `ref_frame_change_method`: keep the reference frame while motion is small.
        static constexpr int32_t REF_FRAME_CHANGE_ON_SMALL_MOTION = 1;
        /// @brief `ref_frame_change_method`: keep the reference frame while inlier support is low.
        static constexpr int32_t REF_FRAME_CHANGE_ON_LOW_INLIERS = 2;

        static constexpr int32_t DEFAULT_REF_FRAME_CHANGE_METHOD = REF_FRAME_CHANGE_ALWAYS;
        static constexpr double DEFAULT_REF_FRAME_MOTION_THRESHOLD_PX = 5.0;
        static constexpr int32_t DEFAULT_REF_FRAME_INLIER_THRESHOLD = 150;
        static constexpr int DEFAULT_SYNC_QUEUE_SIZE = 10;
        static constexpr int DEFAULT_IMAGE_QUEUE_SIZE = 5;

        static inline const std::string DEFAULT_LEFT_IMAGE_TOPIC = "/camera/camera/infra1/image_rect_raw";
        static inline const std::string DEFAULT_RIGHT_IMAGE_TOPIC = "/camera/camera/infra2/image_rect_raw";
        static inline const std::string DEFAULT_LEFT_CAMERA_INFO_TOPIC = "/camera/camera/infra1/camera_info";
        static inline const std::string DEFAULT_RIGHT_CAMERA_INFO_TOPIC = "/camera/camera/infra2/camera_info";
        static inline const std::string DEFAULT_OUTPUT_INFO_TOPIC = "info";
        static inline const std::string DEFAULT_OUTPUT_POINT_CLOUD_TOPIC = "point_cloud";
        static inline const std::string DEFAULT_OUTPUT_INLIERS_TOPIC = "inliers_num";
        static inline const std::string DEFAULT_OUTPUT_TEMPORAL_MATCHES_TOPIC = "temporal_matches_num";

        static inline const std::string DEFAULT_ODOM_FRAME_ID = "odom";
        static inline const std::string DEFAULT_BASE_LINK_FRAME_ID = "base_link";
        static inline const std::string DEFAULT_SENSOR_FRAME_ID = "camera_infra1_optical_frame";

        /// @brief Declares every parameter and copies its value into the matching member.
        /// @return Configuration for the composed OdometryPublisher.
        odometry_publisher_config_t _load_parameters();

        /// @brief Builds the initial integrated pose from the `initial_pose.*` parameters.
        tf2::Transform _load_initial_pose() const;

        /// @brief Constructs the libviso2 odometer from the first camera_info pair received.
        /// @param left_info Left camera calibration.
        /// @param right_info Right camera calibration.
        void _initialize_odometer(
            const sensor_msgs::msg::CameraInfo& left_info,
            const sensor_msgs::msg::CameraInfo& right_info);

        /// @brief Runs one synchronized stereo frame through the odometry pipeline.
        /// @param left_image Left rectified image.
        /// @param right_image Right rectified image.
        /// @param left_info Left camera calibration.
        /// @param right_info Right camera calibration.
        void _synchronized_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
            const sensor_msgs::msg::Image::ConstSharedPtr& right_image,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info);

        /// @brief Feeds a stereo image pair to libviso2 and converts a successful result
        ///        into a delta transform, or a fixed "lost" covariance on failure.
        /// @param left_mono Left rectified image, single-channel 8-bit.
        /// @param right_mono Right rectified image, single-channel 8-bit.
        /// @param delta_transform_out Receives the estimated motion, or identity on failure.
        /// @return True if a motion estimate was produced.
        bool _run_visual_odometry(
            const cv::Mat& left_mono,
            const cv::Mat& right_mono,
            tf2::Transform& delta_transform_out);

        /// @brief Computes the average per-feature pixel flow between the current and
        ///        previous frame, used by the small-motion reference-frame policy.
        /// @param matches Feature matches from the just-completed odometry pass.
        /// @return The average flow in pixels, or 0 if @p matches is empty.
        double _compute_average_feature_flow(const std::vector<Matcher::p_match>& matches) const;

        /// @brief Decides whether the matching reference frame should stay put next pass.
        /// @param motion_estimate_succeeded Whether this pass produced a motion estimate.
        void _update_reference_frame_policy(bool motion_estimate_succeeded);

        /// @brief Publishes a sparse colour point cloud of this pass's inlier feature matches.
        ///
        /// Skipped entirely when nobody is subscribed, since reprojecting every inlier is
        /// the most expensive per-frame operation this node performs.
        /// @param left_info Left camera calibration.
        /// @param left_image Left image, source of the point colours.
        /// @param right_info Right camera calibration.
        void _publish_point_cloud_if_subscribed(
            const sensor_msgs::msg::CameraInfo& left_info,
            const sensor_msgs::msg::Image& left_image,
            const sensor_msgs::msg::CameraInfo& right_info);

        /// @brief Publishes the per-pass diagnostic counters and info message.
        /// @param header Header of the left image the pass was computed from.
        /// @param motion_estimate_valid Whether this pass produced a motion estimate.
        /// @param runtime_s Wall-clock time spent processing this pass, in seconds.
        void _publish_diagnostics(
            const std_msgs::msg::Header& header,
            bool motion_estimate_valid,
            double runtime_s);

        /// @brief Applies runtime updates to the reconfigurable parameters.
        /// @param parameters Parameters being set.
        /// @return Whether the update was accepted, and why if it was not.
        rcl_interfaces::msg::SetParametersResult _parameter_callback(
            const std::vector<rclcpp::Parameter>& parameters);

        // Topics
        std::string _left_image_topic{DEFAULT_LEFT_IMAGE_TOPIC};
        std::string _right_image_topic{DEFAULT_RIGHT_IMAGE_TOPIC};
        std::string _left_camera_info_topic{DEFAULT_LEFT_CAMERA_INFO_TOPIC};
        std::string _right_camera_info_topic{DEFAULT_RIGHT_CAMERA_INFO_TOPIC};
        std::string _output_info_topic{DEFAULT_OUTPUT_INFO_TOPIC};
        std::string _output_point_cloud_topic{DEFAULT_OUTPUT_POINT_CLOUD_TOPIC};
        std::string _output_inliers_topic{DEFAULT_OUTPUT_INLIERS_TOPIC};
        std::string _output_temporal_matches_topic{DEFAULT_OUTPUT_TEMPORAL_MATCHES_TOPIC};
        int _sync_queue_size{DEFAULT_SYNC_QUEUE_SIZE};
        int _image_queue_size{DEFAULT_IMAGE_QUEUE_SIZE};

        // Reference-frame change policy — runtime reconfigurable, so declared atomic.
        std::atomic<int32_t> _ref_frame_change_method{DEFAULT_REF_FRAME_CHANGE_METHOD};
        std::atomic<double> _ref_frame_motion_threshold_px{DEFAULT_REF_FRAME_MOTION_THRESHOLD_PX};
        std::atomic<int32_t> _ref_frame_inlier_threshold{DEFAULT_REF_FRAME_INLIER_THRESHOLD};
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _param_callback_handle;

        // libviso2
        VisualOdometryStereo::parameters _visual_odometer_params{};
        std::unique_ptr<VisualOdometryStereo> _visual_odometer;
        Matrix _reference_motion{Matrix::eye(4)};
        bool _got_lost{false};
        bool _should_change_reference_frame{false};
        int32_t _previous_match_count{0};
        int32_t _previous_inlier_count{0};

        // Pose integration and publishing
        std::unique_ptr<OdometryPublisher> _odometry_publisher;

        // Subscribers and synchronizer
        image_transport::SubscriberFilter _left_image_subscriber;
        image_transport::SubscriberFilter _right_image_subscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> _left_camera_info_subscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> _right_camera_info_subscriber;
        std::shared_ptr<ApproximateSynchronizer> _synchronizer;

        // Diagnostic and point cloud publishers
        rclcpp::Publisher<perseus_interfaces::msg::StereoOdometryInfo>::SharedPtr _info_publisher;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _inliers_count_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _temporal_matches_count_publisher;
    };

}  // namespace perseus_vision
