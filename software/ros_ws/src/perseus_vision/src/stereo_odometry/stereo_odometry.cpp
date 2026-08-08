/// @file stereo_odometry.cpp
/// @brief Implementation of the stereo visual odometry node.

#include "perseus_vision/stereo_odometry/stereo_odometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace perseus_vision
{
    namespace
    {
        /// @brief Queue depth used for the diagnostic and point cloud topics.
        constexpr int QOS_DEPTH = 10;
        /// @brief Minimum gap between repeated "odometer lost" warnings, in milliseconds.
        constexpr int64_t LOST_WARNING_THROTTLE_MS = 10000;
        /// @brief Nanoseconds in one second, used to convert a rate limit into a period.
        constexpr double NANOSECONDS_PER_SECOND = 1e9;

        /// @brief Approximate pose/twist covariance for a successful motion estimate.
        ///
        /// libviso2 does not itself estimate covariance, so these are fixed, hand-tuned
        /// values (0.1 m² linear, ~0.17 rad² i.e. ~10 deg² angular) representing typical
        /// confidence in a good stereo VO estimate, kept from the upstream ROS 1 driver.
        constexpr std::array<double, 36> STANDARD_POSE_COVARIANCE = {
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.17, 0, 0,
            0, 0, 0, 0, 0.17, 0,
            0, 0, 0, 0, 0, 0.17};

        /// @brief Approximate twist covariance for a successful motion estimate.
        constexpr std::array<double, 36> STANDARD_TWIST_COVARIANCE = {
            0.002, 0, 0, 0, 0, 0,
            0, 0.002, 0, 0, 0, 0,
            0, 0, 0.05, 0, 0, 0,
            0, 0, 0, 0.09, 0, 0,
            0, 0, 0, 0, 0.09, 0,
            0, 0, 0, 0, 0, 0.09};

        /// @brief Diagonal-only covariance reported when motion estimation fails, marking
        ///        the published (identity) delta as unreliable rather than confidently zero.
        constexpr double LOST_TRACKING_VARIANCE = 9999.0;

        /// @brief Builds the covariance reported alongside a failed motion estimate.
        std::array<double, 36> make_bad_tracking_covariance()
        {
            std::array<double, 36> covariance{};
            for (int i = 0; i < 6; ++i)
            {
                covariance[static_cast<std::size_t>((i * 6) + i)] = LOST_TRACKING_VARIANCE;
            }
            return covariance;
        }

        /// @brief One inlier feature match reprojected into a coloured 3D point.
        struct colored_point_t
        {
            float x{0.0f};
            float y{0.0f};
            float z{0.0f};
            uint8_t r{0};
            uint8_t g{0};
            uint8_t b{0};
        };

        /// @brief Reads the baseline (in metres) out of a rectified stereo pair's right P matrix.
        ///
        /// Per REP 104, the right camera's projection matrix carries the baseline as
        /// `P[3] = -fx * baseline`. This is the same computation
        /// `image_geometry::StereoCameraModel::baseline()` performs; it is inlined here so
        /// this node does not need an image_geometry dependency for two calibration reads.
        double baseline_from_right_projection(const sensor_msgs::msg::CameraInfo& right_info)
        {
            return (right_info.p[0] != 0.0) ? (-right_info.p[3] / right_info.p[0]) : 0.0;
        }
    }  // namespace

    StereoOdometry::StereoOdometry(const rclcpp::NodeOptions& options)
        : Node("stereo_odometry", options)
    {
        const odometry_publisher_config_t odometry_config = _load_parameters();
        _odometry_publisher = std::make_unique<OdometryPublisher>(*this, odometry_config);

        _info_publisher = create_publisher<perseus_interfaces::msg::StereoOdometryInfo>(
            _output_info_topic, QOS_DEPTH);
        _point_cloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
            _output_point_cloud_topic, QOS_DEPTH);
        _inliers_count_publisher = create_publisher<std_msgs::msg::Int32>(_output_inliers_topic, QOS_DEPTH);
        _temporal_matches_count_publisher =
            create_publisher<std_msgs::msg::Int32>(_output_temporal_matches_topic, QOS_DEPTH);

        rmw_qos_profile_t image_qos = rmw_qos_profile_sensor_data;
        image_qos.depth = static_cast<std::size_t>(_image_queue_size);

        _left_image_subscriber.subscribe(this, _left_image_topic, "raw", image_qos);
        _right_image_subscriber.subscribe(this, _right_image_topic, "raw", image_qos);
        _left_camera_info_subscriber.subscribe(this, _left_camera_info_topic, image_qos);
        _right_camera_info_subscriber.subscribe(this, _right_camera_info_topic, image_qos);

        _synchronizer = std::make_shared<ApproximateSynchronizer>(
            ApproximateSyncPolicy(_sync_queue_size),
            _left_image_subscriber, _right_image_subscriber,
            _left_camera_info_subscriber, _right_camera_info_subscriber);
        _synchronizer->registerCallback(std::bind(
            &StereoOdometry::_synchronized_callback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

        RCLCPP_INFO(get_logger(), "StereoOdometry ready — left='%s' right='%s'",
                    _left_image_topic.c_str(), _right_image_topic.c_str());
    }

    odometry_publisher_config_t StereoOdometry::_load_parameters()
    {
        _left_image_topic = declare_parameter("left_image_topic", DEFAULT_LEFT_IMAGE_TOPIC);
        _right_image_topic = declare_parameter("right_image_topic", DEFAULT_RIGHT_IMAGE_TOPIC);
        _left_camera_info_topic = declare_parameter("left_camera_info_topic", DEFAULT_LEFT_CAMERA_INFO_TOPIC);
        _right_camera_info_topic = declare_parameter("right_camera_info_topic", DEFAULT_RIGHT_CAMERA_INFO_TOPIC);
        _output_info_topic = declare_parameter("output_info_topic", DEFAULT_OUTPUT_INFO_TOPIC);
        _output_point_cloud_topic = declare_parameter("output_point_cloud_topic", DEFAULT_OUTPUT_POINT_CLOUD_TOPIC);
        _output_inliers_topic = declare_parameter("output_inliers_topic", DEFAULT_OUTPUT_INLIERS_TOPIC);
        _output_temporal_matches_topic =
            declare_parameter("output_temporal_matches_topic", DEFAULT_OUTPUT_TEMPORAL_MATCHES_TOPIC);
        _sync_queue_size = declare_parameter("sync_queue_size", DEFAULT_SYNC_QUEUE_SIZE);
        _image_queue_size = declare_parameter("image_queue_size", DEFAULT_IMAGE_QUEUE_SIZE);

        odometry_publisher_config_t odometry_config;
        odometry_config.odometry_topic = declare_parameter("output_odometry_topic", DEFAULT_OUTPUT_ODOMETRY_TOPIC);
        odometry_config.pose_topic = declare_parameter("output_pose_topic", DEFAULT_OUTPUT_POSE_TOPIC);
        odometry_config.odom_frame_id = declare_parameter("odom_frame_id", DEFAULT_ODOM_FRAME_ID);
        odometry_config.base_link_frame_id = declare_parameter("base_link_frame_id", DEFAULT_BASE_LINK_FRAME_ID);
        odometry_config.sensor_frame_id = declare_parameter("sensor_frame_id", DEFAULT_SENSOR_FRAME_ID);
        odometry_config.should_publish_tf = declare_parameter("publish_tf", true);
        odometry_config.should_invert_tf = declare_parameter("invert_tf", false);
        odometry_config.is_initial_pose_in_camera_frame =
            declare_parameter("initial_pose_in_camera_frame", false);

        declare_parameter("initial_pose.tx", 0.0);
        declare_parameter("initial_pose.ty", 0.0);
        declare_parameter("initial_pose.tz", 0.0);
        declare_parameter("initial_pose.qx", 0.0);
        declare_parameter("initial_pose.qy", 0.0);
        declare_parameter("initial_pose.qz", 0.0);
        declare_parameter("initial_pose.qw", 1.0);

        _ref_frame_change_method.store(static_cast<int32_t>(
            declare_parameter("ref_frame_change_method", static_cast<int64_t>(DEFAULT_REF_FRAME_CHANGE_METHOD))));
        _ref_frame_motion_threshold_px.store(
            declare_parameter("ref_frame_motion_threshold", DEFAULT_REF_FRAME_MOTION_THRESHOLD_PX));
        _ref_frame_inlier_threshold.store(static_cast<int32_t>(declare_parameter(
            "ref_frame_inlier_threshold", static_cast<int64_t>(DEFAULT_REF_FRAME_INLIER_THRESHOLD))));
        _processing_frequency_hz.store(
            declare_parameter("processing_frequency_hz", DEFAULT_PROCESSING_FREQUENCY_HZ));
        _static_translation_deadband_m.store(declare_parameter(
            "static_translation_deadband_m", DEFAULT_STATIC_TRANSLATION_DEADBAND_M));
        _static_rotation_deadband_rad.store(
            declare_parameter("static_rotation_deadband_rad", DEFAULT_STATIC_ROTATION_DEADBAND_RAD));

        Matcher::parameters& matcher_params = _visual_odometer_params.match;
        matcher_params.nms_n = static_cast<int32_t>(
            declare_parameter("matcher.nms_n", static_cast<int64_t>(matcher_params.nms_n)));
        matcher_params.nms_tau = static_cast<int32_t>(
            declare_parameter("matcher.nms_tau", static_cast<int64_t>(matcher_params.nms_tau)));
        matcher_params.match_binsize = static_cast<int32_t>(
            declare_parameter("matcher.match_binsize", static_cast<int64_t>(matcher_params.match_binsize)));
        matcher_params.match_radius = static_cast<int32_t>(
            declare_parameter("matcher.match_radius", static_cast<int64_t>(matcher_params.match_radius)));
        matcher_params.match_disp_tolerance = static_cast<int32_t>(declare_parameter(
            "matcher.match_disp_tolerance", static_cast<int64_t>(matcher_params.match_disp_tolerance)));
        matcher_params.outlier_disp_tolerance = static_cast<int32_t>(declare_parameter(
            "matcher.outlier_disp_tolerance", static_cast<int64_t>(matcher_params.outlier_disp_tolerance)));
        matcher_params.outlier_flow_tolerance = static_cast<int32_t>(declare_parameter(
            "matcher.outlier_flow_tolerance", static_cast<int64_t>(matcher_params.outlier_flow_tolerance)));
        matcher_params.multi_stage =
            declare_parameter("matcher.multi_stage", matcher_params.multi_stage != 0) ? 1 : 0;
        matcher_params.half_resolution =
            declare_parameter("matcher.half_resolution", matcher_params.half_resolution != 0) ? 1 : 0;
        matcher_params.refinement = static_cast<int32_t>(
            declare_parameter("matcher.refinement", static_cast<int64_t>(matcher_params.refinement)));

        VisualOdometry::bucketing& bucket_params = _visual_odometer_params.bucket;
        bucket_params.max_features = static_cast<int32_t>(
            declare_parameter("bucket.max_features", static_cast<int64_t>(bucket_params.max_features)));
        bucket_params.bucket_width = declare_parameter("bucket.bucket_width", bucket_params.bucket_width);
        bucket_params.bucket_height = declare_parameter("bucket.bucket_height", bucket_params.bucket_height);

        _visual_odometer_params.ransac_iters = static_cast<int32_t>(declare_parameter(
            "stereo.ransac_iters", static_cast<int64_t>(_visual_odometer_params.ransac_iters)));
        _visual_odometer_params.inlier_threshold =
            declare_parameter("stereo.inlier_threshold", _visual_odometer_params.inlier_threshold);
        _visual_odometer_params.reweighting =
            declare_parameter("stereo.reweighting", _visual_odometer_params.reweighting);

        _param_callback_handle = add_on_set_parameters_callback(
            std::bind(&StereoOdometry::_parameter_callback, this, std::placeholders::_1));

        return odometry_config;
    }

    tf2::Transform StereoOdometry::_load_initial_pose() const
    {
        tf2::Transform initial_pose;
        initial_pose.setOrigin(tf2::Vector3(
            get_parameter("initial_pose.tx").as_double(),
            get_parameter("initial_pose.ty").as_double(),
            get_parameter("initial_pose.tz").as_double()));
        initial_pose.setRotation(tf2::Quaternion(
            get_parameter("initial_pose.qx").as_double(),
            get_parameter("initial_pose.qy").as_double(),
            get_parameter("initial_pose.qz").as_double(),
            get_parameter("initial_pose.qw").as_double()));
        return initial_pose;
    }

    void StereoOdometry::_initialize_odometer(
        const sensor_msgs::msg::CameraInfo& left_info,
        const sensor_msgs::msg::CameraInfo& right_info)
    {
        // Both images are already rectified onto a common image plane, so K's focal
        // length and principal point apply equally to the left and right cameras.
        _visual_odometer_params.calib.f = left_info.k[0];
        _visual_odometer_params.calib.cu = left_info.k[2];
        _visual_odometer_params.calib.cv = left_info.k[5];
        _visual_odometer_params.base = baseline_from_right_projection(right_info);

        _visual_odometer = std::make_unique<VisualOdometryStereo>(_visual_odometer_params);

        if (!left_info.header.frame_id.empty())
        {
            _odometry_publisher->set_sensor_frame_id(left_info.header.frame_id);
        }

        RCLCPP_INFO(get_logger(),
                    "Initialized libviso2 stereo odometry — f=%.2f cu=%.2f cv=%.2f base=%.4fm "
                    "ref_frame_change_method=%d",
                    _visual_odometer_params.calib.f, _visual_odometer_params.calib.cu,
                    _visual_odometer_params.calib.cv, _visual_odometer_params.base,
                    _ref_frame_change_method.load());

        _odometry_publisher->set_initial_pose(_load_initial_pose());
    }

    bool StereoOdometry::_should_process_frame_now()
    {
        const double processing_frequency_hz = _processing_frequency_hz.load();
        if (processing_frequency_hz <= 0.0)
        {
            return true;
        }

        const int64_t now_ns = this->now().nanoseconds();
        // Reset the rate limiter if the clock jumped backwards, such as on a sim time reset.
        if (_last_processed_time_ns != 0 && now_ns < _last_processed_time_ns)
        {
            _last_processed_time_ns = 0;
        }

        const int64_t min_period_ns = static_cast<int64_t>(NANOSECONDS_PER_SECOND / processing_frequency_hz);
        if (_last_processed_time_ns != 0 && (now_ns - _last_processed_time_ns) < min_period_ns)
        {
            return false;
        }

        _last_processed_time_ns = now_ns;
        return true;
    }

    void StereoOdometry::_synchronized_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_image,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info)
    {
        const auto start_time = std::chrono::steady_clock::now();

        const bool is_first_run = (_visual_odometer == nullptr);
        if (is_first_run)
        {
            _initialize_odometer(*left_info, *right_info);
        }
        else if (!_should_process_frame_now())
        {
            return;
        }

        cv_bridge::CvImageConstPtr left_mono;
        cv_bridge::CvImageConstPtr right_mono;
        try
        {
            left_mono = cv_bridge::toCvShare(left_image, sensor_msgs::image_encodings::MONO8);
            right_mono = cv_bridge::toCvShare(right_image, sensor_msgs::image_encodings::MONO8);
        }
        catch (const cv_bridge::Exception& error)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", error.what());
            return;
        }

        if (left_mono->image.step[0] != right_mono->image.step[0] ||
            left_image->width != right_image->width || left_image->height != right_image->height)
        {
            RCLCPP_ERROR(get_logger(), "Left and right images must have matching dimensions");
            return;
        }

        bool motion_estimate_valid = false;

        if (is_first_run || _got_lost)
        {
            // Only feed the odometer's ring buffer; no correspondence exists yet to
            // estimate motion from, so nothing is published on a got_lost recovery frame.
            int32_t dims[] = {
                static_cast<int32_t>(left_image->width), static_cast<int32_t>(left_image->height),
                static_cast<int32_t>(left_mono->image.step[0])};
            _visual_odometer->process(left_mono->image.data, right_mono->image.data, dims);
            _got_lost = false;

            if (is_first_run)
            {
                tf2::Transform identity_transform;
                identity_transform.setIdentity();
                _odometry_publisher->integrate_and_publish(identity_transform, left_image->header.stamp);
            }
        }
        else
        {
            tf2::Transform delta_transform;
            motion_estimate_valid = _run_visual_odometry(left_mono->image, right_mono->image, delta_transform);
            if (motion_estimate_valid)
            {
                _suppress_negligible_motion(delta_transform);
            }
            _odometry_publisher->integrate_and_publish(delta_transform, left_image->header.stamp);

            if (motion_estimate_valid)
            {
                _publish_point_cloud_if_subscribed(*left_info, *left_image, *right_info);
            }

            _update_reference_frame_policy(motion_estimate_valid);
        }

        const double runtime_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        _publish_diagnostics(left_image->header, motion_estimate_valid, runtime_s);
    }

    bool StereoOdometry::_run_visual_odometry(
        const cv::Mat& left_mono,
        const cv::Mat& right_mono,
        tf2::Transform& delta_transform_out)
    {
        delta_transform_out.setIdentity();

        int32_t dims[] = {left_mono.cols, left_mono.rows, static_cast<int32_t>(left_mono.step[0])};
        const bool success = _visual_odometer->process(
            left_mono.data, right_mono.data, dims, _should_change_reference_frame);

        if (!success)
        {
            _odometry_publisher->set_pose_covariance(make_bad_tracking_covariance());
            _odometry_publisher->set_twist_covariance(make_bad_tracking_covariance());
            RCLCPP_DEBUG(get_logger(), "VisualOdometryStereo::process() failed");
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), LOST_WARNING_THROTTLE_MS,
                                 "Visual odometer got lost!");
            _got_lost = true;
            return false;
        }

        const Matrix motion = Matrix::inv(_visual_odometer->getMotion());
        RCLCPP_DEBUG(get_logger(), "Found %d matches with %d inliers",
                     _visual_odometer->getNumberOfMatches(), _visual_odometer->getNumberOfInliers());

        // If the last pass kept the previous frame as the matching reference (small
        // motion), the odometer reports motion relative to that older reference, so the
        // reference's own motion has to be subtracted back out to get this pass's increment.
        const Matrix camera_motion =
            _should_change_reference_frame ? (Matrix::inv(_reference_motion) * motion) : motion;
        _reference_motion = motion;

        const tf2::Matrix3x3 rotation(
            camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
            camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
            camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        const tf2::Vector3 translation(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        delta_transform_out = tf2::Transform(rotation, translation);

        _odometry_publisher->set_pose_covariance(STANDARD_POSE_COVARIANCE);
        _odometry_publisher->set_twist_covariance(STANDARD_TWIST_COVARIANCE);
        return true;
    }

    void StereoOdometry::_suppress_negligible_motion(tf2::Transform& delta_transform) const
    {
        const double translation_deadband_m = _static_translation_deadband_m.load();
        const double rotation_deadband_rad = _static_rotation_deadband_rad.load();
        if (translation_deadband_m <= 0.0 && rotation_deadband_rad <= 0.0)
        {
            return;
        }

        // A deadband left at its disabled (<=0) default does not require that axis to
        // pass -- otherwise configuring only one of the two would make the other's
        // always-false "< 0" comparison block suppression entirely.
        const bool translation_is_negligible =
            translation_deadband_m <= 0.0 || delta_transform.getOrigin().length() < translation_deadband_m;
        const bool rotation_is_negligible =
            rotation_deadband_rad <= 0.0 ||
            std::fabs(delta_transform.getRotation().getAngle()) < rotation_deadband_rad;

        if (translation_is_negligible && rotation_is_negligible)
        {
            delta_transform.setIdentity();
        }
    }

    double StereoOdometry::_compute_average_feature_flow(const std::vector<Matcher::p_match>& matches) const
    {
        if (matches.empty())
        {
            return 0.0;
        }

        double total_flow = 0.0;
        for (const auto& match : matches)
        {
            const double delta_u = match.u1c - match.u1p;
            const double delta_v = match.v1c - match.v1p;
            total_flow += std::sqrt((delta_u * delta_u) + (delta_v * delta_v));
        }
        return total_flow / static_cast<double>(matches.size());
    }

    void StereoOdometry::_update_reference_frame_policy(bool motion_estimate_succeeded)
    {
        if (!motion_estimate_succeeded)
        {
            _should_change_reference_frame = false;
            return;
        }

        switch (_ref_frame_change_method.load())
        {
        case REF_FRAME_CHANGE_ON_SMALL_MOTION:
        {
            const double feature_flow_px = _compute_average_feature_flow(_visual_odometer->getMatches());
            _should_change_reference_frame = feature_flow_px < _ref_frame_motion_threshold_px.load();
            RCLCPP_DEBUG(get_logger(), "Feature flow is %.2fpx, marking last motion as %s",
                         feature_flow_px, _should_change_reference_frame ? "small" : "normal");
            break;
        }
        case REF_FRAME_CHANGE_ON_LOW_INLIERS:
            _should_change_reference_frame =
                _visual_odometer->getNumberOfInliers() > _ref_frame_inlier_threshold.load();
            break;
        case REF_FRAME_CHANGE_ALWAYS:
        default:
            _should_change_reference_frame = false;
            break;
        }
    }

    void StereoOdometry::_publish_point_cloud_if_subscribed(
        const sensor_msgs::msg::CameraInfo& left_info,
        const sensor_msgs::msg::Image& left_image,
        const sensor_msgs::msg::CameraInfo& right_info)
    {
        if (_point_cloud_publisher->get_subscription_count() == 0)
        {
            return;
        }

        cv_bridge::CvImageConstPtr color_image;
        try
        {
            color_image = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::RGB8);
        }
        catch (const cv_bridge::Exception& error)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", error.what());
            return;
        }

        const double focal_length_px = left_info.k[0];
        const double principal_point_u_px = left_info.k[2];
        const double principal_point_v_px = left_info.k[5];
        const double baseline_m = baseline_from_right_projection(right_info);

        const std::vector<Matcher::p_match> matches = _visual_odometer->getMatches();
        const std::vector<int32_t> inlier_indices = _visual_odometer->getInlierIndices();

        std::vector<colored_point_t> points;
        points.reserve(inlier_indices.size());
        for (const int32_t inlier_index : inlier_indices)
        {
            const Matcher::p_match& match = matches[static_cast<std::size_t>(inlier_index)];
            const double disparity = match.u1c - match.u2c;
            const int u = static_cast<int>(match.u1c);
            const int v = static_cast<int>(match.v1c);
            if (disparity <= 0.0 || u < 0 || v < 0 || u >= color_image->image.cols || v >= color_image->image.rows)
            {
                continue;
            }

            const double depth_m = (focal_length_px * baseline_m) / disparity;
            const cv::Vec3b color = color_image->image.at<cv::Vec3b>(v, u);

            colored_point_t point;
            point.x = static_cast<float>((match.u1c - principal_point_u_px) * depth_m / focal_length_px);
            point.y = static_cast<float>((match.v1c - principal_point_v_px) * depth_m / focal_length_px);
            point.z = static_cast<float>(depth_m);
            point.r = color[0];
            point.g = color[1];
            point.b = color[2];
            points.push_back(point);
        }

        sensor_msgs::msg::PointCloud2 cloud_message;
        cloud_message.header.frame_id = _odometry_publisher->get_sensor_frame_id();
        cloud_message.header.stamp = left_image.header.stamp;

        sensor_msgs::PointCloud2Modifier modifier(cloud_message);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_message, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_message, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_message, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_message, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_message, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_message, "b");
        for (const colored_point_t& point : points)
        {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            *iter_r = point.r;
            *iter_g = point.g;
            *iter_b = point.b;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }

        RCLCPP_DEBUG(get_logger(), "Publishing point cloud with %zu points", points.size());
        _point_cloud_publisher->publish(cloud_message);
    }

    void StereoOdometry::_publish_diagnostics(
        const std_msgs::msg::Header& header,
        bool motion_estimate_valid,
        double runtime_s)
    {
        const int32_t current_match_count = _visual_odometer->getNumberOfMatches();
        const int32_t current_inlier_count = _visual_odometer->getNumberOfInliers();

        // While lost and the counts have not moved since the last pass, report zero
        // rather than a stale count left over from before tracking was lost.
        const int32_t reported_match_count =
            (_got_lost && current_match_count == _previous_match_count) ? 0 : current_match_count;
        const int32_t reported_inlier_count =
            (_got_lost && current_inlier_count == _previous_inlier_count) ? 0 : current_inlier_count;

        if (_temporal_matches_count_publisher->get_subscription_count() > 0)
        {
            std_msgs::msg::Int32 matches_message;
            matches_message.data = reported_match_count;
            _temporal_matches_count_publisher->publish(matches_message);
        }

        if (_inliers_count_publisher->get_subscription_count() > 0)
        {
            std_msgs::msg::Int32 inliers_message;
            inliers_message.data = reported_inlier_count;
            _inliers_count_publisher->publish(inliers_message);
        }

        if (_info_publisher->get_subscription_count() > 0)
        {
            perseus_interfaces::msg::StereoOdometryInfo info_message;
            info_message.header = header;
            info_message.got_lost = _got_lost;
            info_message.change_reference_frame = _should_change_reference_frame;
            info_message.motion_estimate_valid = motion_estimate_valid;
            info_message.num_matches = reported_match_count;
            info_message.num_inliers = reported_inlier_count;
            info_message.runtime_s = runtime_s;
            _info_publisher->publish(info_message);
        }

        _previous_match_count = current_match_count;
        _previous_inlier_count = current_inlier_count;
    }

    rcl_interfaces::msg::SetParametersResult StereoOdometry::_parameter_callback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& parameter : parameters)
        {
            if (parameter.get_name() == "ref_frame_change_method")
            {
                const int64_t method = parameter.as_int();
                if (method < REF_FRAME_CHANGE_ALWAYS || method > REF_FRAME_CHANGE_ON_LOW_INLIERS)
                {
                    result.successful = false;
                    result.reason = "ref_frame_change_method must be 0, 1, or 2";
                    return result;
                }
                _ref_frame_change_method.store(static_cast<int32_t>(method));
                continue;
            }

            if (parameter.get_name() == "ref_frame_motion_threshold")
            {
                _ref_frame_motion_threshold_px.store(parameter.as_double());
                continue;
            }

            if (parameter.get_name() == "ref_frame_inlier_threshold")
            {
                _ref_frame_inlier_threshold.store(static_cast<int32_t>(parameter.as_int()));
                continue;
            }

            if (parameter.get_name() == "processing_frequency_hz")
            {
                const double processing_frequency_hz = parameter.as_double();
                if (processing_frequency_hz < 0.0)
                {
                    result.successful = false;
                    result.reason = "processing_frequency_hz must be non-negative";
                    return result;
                }
                _processing_frequency_hz.store(processing_frequency_hz);
                continue;
            }

            if (parameter.get_name() == "static_translation_deadband_m")
            {
                const double deadband_m = parameter.as_double();
                if (deadband_m < 0.0)
                {
                    result.successful = false;
                    result.reason = "static_translation_deadband_m must be non-negative";
                    return result;
                }
                _static_translation_deadband_m.store(deadband_m);
                continue;
            }

            if (parameter.get_name() == "static_rotation_deadband_rad")
            {
                const double deadband_rad = parameter.as_double();
                if (deadband_rad < 0.0)
                {
                    result.successful = false;
                    result.reason = "static_rotation_deadband_rad must be non-negative";
                    return result;
                }
                _static_rotation_deadband_rad.store(deadband_rad);
                continue;
            }
        }

        return result;
    }

}  // namespace perseus_vision

RCLCPP_COMPONENTS_REGISTER_NODE(perseus_vision::StereoOdometry)
