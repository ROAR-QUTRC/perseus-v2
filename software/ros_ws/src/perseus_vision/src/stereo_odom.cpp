#include "perseus_vision/stereo_odom.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>

#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace perseus_vision
{

    namespace
    {
        double rotation_angle_from_matrix(const cv::Matx33d& rotation)
        {
            const double trace = rotation(0, 0) + rotation(1, 1) + rotation(2, 2);
            const double cos_angle = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
            return std::acos(cos_angle);
        }
    }  // namespace

    StereoOdom::StereoOdom()
        : Node("stereo_odom")
    {
        left_image_topic_ = declare_parameter<std::string>(
            "left_image_topic", "/camera/camera/infra1/image_rect_raw");
        right_image_topic_ = declare_parameter<std::string>(
            "right_image_topic", "/camera/camera/infra2/image_rect_raw");
        left_camera_info_topic_ = declare_parameter<std::string>(
            "left_camera_info_topic", "/camera/camera/infra1/camera_info");
        right_camera_info_topic_ = declare_parameter<std::string>(
            "right_camera_info_topic", "/camera/camera/infra2/camera_info");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/perseus_vision/stereo_odom");
        odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
        child_frame_id_ = declare_parameter<std::string>("child_frame_id", "camera_infra1_optical_frame");

        publish_tf_ = declare_parameter<bool>("publish_tf", true);
        use_clahe_ = declare_parameter<bool>("use_clahe", true);
        use_equalize_hist_ = declare_parameter<bool>("use_equalize_hist", false);
        sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
        max_features_ = declare_parameter<int>("max_features", 250);
        min_tracked_features_ = declare_parameter<int>("min_tracked_features", 80);
        feature_quality_level_ = declare_parameter<double>("feature_quality_level", 0.01);
        min_feature_distance_px_ = declare_parameter<double>("min_feature_distance_px", 12.0);
        feature_block_size_ = declare_parameter<int>("feature_block_size", 5);
        lk_window_size_px_ = declare_parameter<int>("lk_window_size_px", 21);
        lk_max_pyramid_level_ = declare_parameter<int>("lk_max_pyramid_level", 3);
        min_inliers_ = declare_parameter<int>("min_inliers", 20);
        ransac_iterations_ = declare_parameter<int>("ransac_iterations", 120);
        ransac_inlier_threshold_m_ = declare_parameter<double>("ransac_inlier_threshold_m", 0.10);
        max_frame_delta_s_ = declare_parameter<double>("max_frame_delta_s", 0.25);
        max_valid_depth_m_ = declare_parameter<double>("max_valid_depth_m", 6.0);
        min_valid_depth_m_ = declare_parameter<double>("min_valid_depth_m", 0.2);
        min_valid_disparity_px_ = declare_parameter<double>("min_valid_disparity_px", 1.0);
        max_translation_per_frame_m_ = declare_parameter<double>("max_translation_per_frame_m", 0.50);
        max_rotation_per_frame_rad_ = declare_parameter<double>("max_rotation_per_frame_rad", 0.70);
        stereo_num_disparities_ = declare_parameter<int>("stereo_num_disparities", 64);
        stereo_block_size_ = declare_parameter<int>("stereo_block_size", 15);
        stereo_pre_filter_cap_ = declare_parameter<int>("stereo_pre_filter_cap", 31);
        stereo_texture_threshold_ = declare_parameter<int>("stereo_texture_threshold", 10);
        stereo_uniqueness_ratio_ = declare_parameter<int>("stereo_uniqueness_ratio", 10);
        stereo_speckle_window_size_ = declare_parameter<int>("stereo_speckle_window_size", 50);
        stereo_speckle_range_ = declare_parameter<int>("stereo_speckle_range", 2);
        stereo_disp12_max_diff_ = declare_parameter<int>("stereo_disp12_max_diff", 1);

        stereo_num_disparities_ = std::max(16, stereo_num_disparities_);
        if (stereo_num_disparities_ % 16 != 0)
        {
            stereo_num_disparities_ = ((stereo_num_disparities_ / 16) + 1) * 16;
        }
        if (stereo_block_size_ % 2 == 0)
        {
            stereo_block_size_ += 1;
        }
        stereo_block_size_ = std::max(5, stereo_block_size_);

        clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
        stereo_matcher_ = cv::StereoBM::create(stereo_num_disparities_, stereo_block_size_);
        stereo_matcher_->setPreFilterCap(stereo_pre_filter_cap_);
        stereo_matcher_->setTextureThreshold(stereo_texture_threshold_);
        stereo_matcher_->setUniquenessRatio(stereo_uniqueness_ratio_);
        stereo_matcher_->setSpeckleWindowSize(stereo_speckle_window_size_);
        stereo_matcher_->setSpeckleRange(stereo_speckle_range_);
        stereo_matcher_->setDisp12MaxDiff(stereo_disp12_max_diff_);

        left_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            left_camera_info_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&StereoOdom::left_camera_info_callback, this, std::placeholders::_1));
        right_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            right_camera_info_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&StereoOdom::right_camera_info_callback, this, std::placeholders::_1));

        left_image_sub_.subscribe(this, left_image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
        right_image_sub_.subscribe(this, right_image_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
        stereo_sync_ = std::make_shared<ImageSynchronizer>(ImageSyncPolicy(sync_queue_size_), left_image_sub_, right_image_sub_);
        stereo_sync_->registerCallback(std::bind(
            &StereoOdom::stereo_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(
            get_logger(),
            "Stereo odom listening on [%s] and [%s], publishing odom on [%s]",
            left_image_topic_.c_str(),
            right_image_topic_.c_str(),
            odom_topic_.c_str());
    }

    void StereoOdom::left_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::scoped_lock lock(camera_model_mutex_);
        latest_left_camera_info_ = msg;
        update_camera_model_locked(*latest_left_camera_info_, latest_right_camera_info_.get());
    }

    void StereoOdom::right_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::scoped_lock lock(camera_model_mutex_);
        latest_right_camera_info_ = msg;
        if (latest_left_camera_info_ != nullptr)
        {
            update_camera_model_locked(*latest_left_camera_info_, latest_right_camera_info_.get());
        }
    }

    bool StereoOdom::update_camera_model_locked(
        const sensor_msgs::msg::CameraInfo& left_info,
        const sensor_msgs::msg::CameraInfo* right_info)
    {
        if (left_info.k[0] <= 0.0 || left_info.k[4] <= 0.0)
        {
            return false;
        }

        camera_model_.fx = left_info.k[0];
        camera_model_.fy = left_info.k[4];
        camera_model_.cx = left_info.k[2];
        camera_model_.cy = left_info.k[5];

        double baseline_m = 0.0;
        if (right_info != nullptr && right_info->p[3] != 0.0)
        {
            baseline_m = std::abs(right_info->p[3] / right_info->p[0]);
        }
        else if (left_info.p[3] != 0.0)
        {
            baseline_m = std::abs(left_info.p[3] / left_info.p[0]);
        }

        camera_model_.baseline_m = baseline_m;
        camera_model_.valid = camera_model_.baseline_m > 1e-6;

        if (!camera_model_.valid)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Stereo odom is waiting for valid camera baseline from CameraInfo.");
        }

        return camera_model_.valid;
    }

    cv::Mat StereoOdom::prepare_grayscale_image(const sensor_msgs::msg::Image& msg) const
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (const cv_bridge::Exception& e)
        {
            throw std::runtime_error("cv_bridge conversion failed: " + std::string(e.what()));
        }

        cv::Mat image = cv_ptr->image.clone();
        if (use_clahe_)
        {
            clahe_->apply(image, image);
        }
        else if (use_equalize_hist_)
        {
            cv::equalizeHist(image, image);
        }

        return image;
    }

    cv::Mat StereoOdom::compute_disparity(const cv::Mat& left_image, const cv::Mat& right_image)
    {
        cv::Mat disparity_16s;
        stereo_matcher_->compute(left_image, right_image, disparity_16s);

        cv::Mat disparity_f32;
        disparity_16s.convertTo(disparity_f32, CV_32F, 1.0 / 16.0);
        return disparity_f32;
    }

    std::vector<cv::Point2f> StereoOdom::detect_features(const cv::Mat& image, const cv::Mat& disparity) const
    {
        std::vector<cv::Point2f> candidates;
        cv::goodFeaturesToTrack(
            image,
            candidates,
            max_features_,
            feature_quality_level_,
            min_feature_distance_px_,
            cv::Mat(),
            feature_block_size_,
            false,
            0.04);

        std::vector<cv::Point2f> filtered;
        filtered.reserve(candidates.size());
        for (const auto& point : candidates)
        {
            cv::Vec3d unused_point_3d;
            if (reproject_to_3d(point, disparity, unused_point_3d))
            {
                filtered.push_back(point);
            }
        }

        return filtered;
    }

    bool StereoOdom::reproject_to_3d(const cv::Point2f& pixel, const cv::Mat& disparity, cv::Vec3d& point_3d) const
    {
        std::scoped_lock lock(camera_model_mutex_);
        if (!camera_model_.valid)
        {
            return false;
        }

        const int u = static_cast<int>(std::lround(pixel.x));
        const int v = static_cast<int>(std::lround(pixel.y));
        if (u < 0 || v < 0 || u >= disparity.cols || v >= disparity.rows)
        {
            return false;
        }

        const float disparity_px = disparity.at<float>(v, u);
        if (!std::isfinite(disparity_px) || disparity_px < min_valid_disparity_px_)
        {
            return false;
        }

        const double depth_m = (camera_model_.fx * camera_model_.baseline_m) / static_cast<double>(disparity_px);
        if (!std::isfinite(depth_m) || depth_m < min_valid_depth_m_ || depth_m > max_valid_depth_m_)
        {
            return false;
        }

        const double x = (pixel.x - camera_model_.cx) * depth_m / camera_model_.fx;
        const double y = (pixel.y - camera_model_.cy) * depth_m / camera_model_.fy;
        point_3d = cv::Vec3d(x, y, depth_m);
        return true;
    }

    bool StereoOdom::estimate_rigid_transform(
        const std::vector<cv::Vec3d>& previous_points,
        const std::vector<cv::Vec3d>& current_points,
        const std::vector<int>& indices,
        cv::Matx33d& rotation,
        cv::Vec3d& translation,
        double& mean_error_m) const
    {
        if (indices.size() < 3)
        {
            return false;
        }

        cv::Vec3d previous_centroid(0.0, 0.0, 0.0);
        cv::Vec3d current_centroid(0.0, 0.0, 0.0);
        for (const int index : indices)
        {
            previous_centroid += previous_points[static_cast<size_t>(index)];
            current_centroid += current_points[static_cast<size_t>(index)];
        }
        previous_centroid *= 1.0 / static_cast<double>(indices.size());
        current_centroid *= 1.0 / static_cast<double>(indices.size());

        cv::Mat covariance = cv::Mat::zeros(3, 3, CV_64F);
        for (const int index : indices)
        {
            const cv::Vec3d previous_centered = previous_points[static_cast<size_t>(index)] - previous_centroid;
            const cv::Vec3d current_centered = current_points[static_cast<size_t>(index)] - current_centroid;
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    covariance.at<double>(row, col) += previous_centered[row] * current_centered[col];
                }
            }
        }

        cv::SVD svd(covariance, cv::SVD::FULL_UV);
        cv::Mat rotation_mat = svd.vt.t() * svd.u.t();
        if (cv::determinant(rotation_mat) < 0.0)
        {
            cv::Mat vt = svd.vt.clone();
            vt.row(2) *= -1.0;
            rotation_mat = vt.t() * svd.u.t();
        }

        rotation = cv::Matx33d(
            rotation_mat.at<double>(0, 0),
            rotation_mat.at<double>(0, 1),
            rotation_mat.at<double>(0, 2),
            rotation_mat.at<double>(1, 0),
            rotation_mat.at<double>(1, 1),
            rotation_mat.at<double>(1, 2),
            rotation_mat.at<double>(2, 0),
            rotation_mat.at<double>(2, 1),
            rotation_mat.at<double>(2, 2));

        translation = current_centroid - (rotation * previous_centroid);

        double total_error = 0.0;
        for (const int index : indices)
        {
            const cv::Vec3d residual =
                (rotation * previous_points[static_cast<size_t>(index)] + translation) -
                current_points[static_cast<size_t>(index)];
            total_error += cv::norm(residual);
        }
        mean_error_m = total_error / static_cast<double>(indices.size());
        return std::isfinite(mean_error_m);
    }

    StereoOdom::RigidTransform StereoOdom::estimate_motion_ransac(
        const std::vector<cv::Vec3d>& previous_points,
        const std::vector<cv::Vec3d>& current_points) const
    {
        RigidTransform best_transform;
        if (previous_points.size() != current_points.size() || previous_points.size() < static_cast<size_t>(min_inliers_))
        {
            return best_transform;
        }

        std::vector<int> all_indices(previous_points.size());
        std::iota(all_indices.begin(), all_indices.end(), 0);

        std::mt19937 random_engine(static_cast<unsigned int>(this->now().nanoseconds()));
        for (int iteration = 0; iteration < ransac_iterations_; ++iteration)
        {
            std::shuffle(all_indices.begin(), all_indices.end(), random_engine);
            const std::vector<int> sample_indices = {all_indices[0], all_indices[1], all_indices[2]};

            cv::Matx33d candidate_rotation;
            cv::Vec3d candidate_translation;
            double candidate_error = 0.0;
            if (!estimate_rigid_transform(
                    previous_points,
                    current_points,
                    sample_indices,
                    candidate_rotation,
                    candidate_translation,
                    candidate_error))
            {
                continue;
            }

            std::vector<int> inliers;
            inliers.reserve(previous_points.size());
            for (size_t i = 0; i < previous_points.size(); ++i)
            {
                const cv::Vec3d residual =
                    (candidate_rotation * previous_points[i] + candidate_translation) - current_points[i];
                if (cv::norm(residual) < ransac_inlier_threshold_m_)
                {
                    inliers.push_back(static_cast<int>(i));
                }
            }

            if (inliers.size() < static_cast<size_t>(min_inliers_) ||
                inliers.size() <= best_transform.inlier_indices.size())
            {
                continue;
            }

            cv::Matx33d refined_rotation;
            cv::Vec3d refined_translation;
            double refined_error = 0.0;
            if (!estimate_rigid_transform(
                    previous_points,
                    current_points,
                    inliers,
                    refined_rotation,
                    refined_translation,
                    refined_error))
            {
                continue;
            }

            best_transform.valid = true;
            best_transform.rotation = refined_rotation;
            best_transform.translation = refined_translation;
            best_transform.inlier_indices = std::move(inliers);
            best_transform.mean_error_m = refined_error;
        }

        return best_transform;
    }

    void StereoOdom::reset_tracking(const cv::Mat& left_image, const cv::Mat& disparity, const rclcpp::Time& stamp)
    {
        previous_frame_.stamp = stamp;
        previous_frame_.left_image = left_image.clone();
        previous_frame_.disparity = disparity.clone();
        previous_frame_.features = detect_features(left_image, disparity);
        has_previous_frame_ = !previous_frame_.features.empty();

        if (!has_previous_frame_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Stereo odom could not seed enough valid stereo features.");
        }
    }

    void StereoOdom::publish_odometry(
        const rclcpp::Time& stamp,
        const cv::Matx33d& delta_rotation,
        const cv::Vec3d& delta_translation,
        double dt_s)
    {
        const cv::Matx33d camera_rotation_from_previous = delta_rotation.t();
        const cv::Vec3d camera_translation_from_previous = -(camera_rotation_from_previous * delta_translation);

        world_translation_from_camera_ += world_rotation_from_camera_ * camera_translation_from_previous;
        world_rotation_from_camera_ = world_rotation_from_camera_ * camera_rotation_from_previous;

        tf2::Matrix3x3 rotation_matrix(
            world_rotation_from_camera_(0, 0),
            world_rotation_from_camera_(0, 1),
            world_rotation_from_camera_(0, 2),
            world_rotation_from_camera_(1, 0),
            world_rotation_from_camera_(1, 1),
            world_rotation_from_camera_(1, 2),
            world_rotation_from_camera_(2, 0),
            world_rotation_from_camera_(2, 1),
            world_rotation_from_camera_(2, 2));
        tf2::Quaternion quaternion;
        rotation_matrix.getRotation(quaternion);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = child_frame_id_;
        odom_msg.pose.pose.position.x = world_translation_from_camera_[0];
        odom_msg.pose.pose.position.y = world_translation_from_camera_[1];
        odom_msg.pose.pose.position.z = world_translation_from_camera_[2];
        odom_msg.pose.pose.orientation = tf2::toMsg(quaternion);

        if (dt_s > 1e-3)
        {
            odom_msg.twist.twist.linear.x = camera_translation_from_previous[0] / dt_s;
            odom_msg.twist.twist.linear.y = camera_translation_from_previous[1] / dt_s;
            odom_msg.twist.twist.linear.z = camera_translation_from_previous[2] / dt_s;

            cv::Mat rotation_vector;
            cv::Rodrigues(camera_rotation_from_previous, rotation_vector);
            odom_msg.twist.twist.angular.x = rotation_vector.at<double>(0, 0) / dt_s;
            odom_msg.twist.twist.angular.y = rotation_vector.at<double>(1, 0) / dt_s;
            odom_msg.twist.twist.angular.z = rotation_vector.at<double>(2, 0) / dt_s;
        }

        odom_msg.pose.covariance[0] = 0.05;
        odom_msg.pose.covariance[7] = 0.05;
        odom_msg.pose.covariance[14] = 0.10;
        odom_msg.pose.covariance[21] = 0.10;
        odom_msg.pose.covariance[28] = 0.10;
        odom_msg.pose.covariance[35] = 0.10;

        odom_pub_->publish(odom_msg);

        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped transform_msg;
            transform_msg.header = odom_msg.header;
            transform_msg.child_frame_id = child_frame_id_;
            transform_msg.transform.translation.x = odom_msg.pose.pose.position.x;
            transform_msg.transform.translation.y = odom_msg.pose.pose.position.y;
            transform_msg.transform.translation.z = odom_msg.pose.pose.position.z;
            transform_msg.transform.rotation = odom_msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(transform_msg);
        }
    }

    void StereoOdom::stereo_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
    {
        CameraModel current_camera_model;
        {
            std::scoped_lock lock(camera_model_mutex_);
            current_camera_model = camera_model_;
        }

        if (!current_camera_model.valid)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Stereo odom is waiting for rectified CameraInfo with a non-zero stereo baseline.");
            return;
        }

        cv::Mat left_image;
        cv::Mat right_image;
        try
        {
            left_image = prepare_grayscale_image(*left_msg);
            right_image = prepare_grayscale_image(*right_msg);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Stereo odom image conversion failed: %s",
                e.what());
            return;
        }

        if (left_image.empty() || right_image.empty())
        {
            return;
        }

        const cv::Mat disparity = compute_disparity(left_image, right_image);
        const rclcpp::Time stamp(left_msg->header.stamp);

        if (!has_previous_frame_)
        {
            reset_tracking(left_image, disparity, stamp);
            return;
        }

        const double dt_s = (stamp - previous_frame_.stamp).seconds();
        if (dt_s <= 0.0 || dt_s > max_frame_delta_s_)
        {
            reset_tracking(left_image, disparity, stamp);
            return;
        }

        std::vector<cv::Point2f> tracked_points;
        std::vector<unsigned char> tracking_status;
        std::vector<float> tracking_errors;
        cv::calcOpticalFlowPyrLK(
            previous_frame_.left_image,
            left_image,
            previous_frame_.features,
            tracked_points,
            tracking_status,
            tracking_errors,
            cv::Size(lk_window_size_px_, lk_window_size_px_),
            lk_max_pyramid_level_,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01));

        std::vector<cv::Vec3d> previous_points_3d;
        std::vector<cv::Vec3d> current_points_3d;
        std::vector<cv::Point2f> surviving_points;
        previous_points_3d.reserve(tracked_points.size());
        current_points_3d.reserve(tracked_points.size());
        surviving_points.reserve(tracked_points.size());

        for (size_t i = 0; i < tracked_points.size(); ++i)
        {
            if (tracking_status[i] == 0)
            {
                continue;
            }

            if (tracked_points[i].x < 0.0F || tracked_points[i].y < 0.0F ||
                tracked_points[i].x >= static_cast<float>(left_image.cols) ||
                tracked_points[i].y >= static_cast<float>(left_image.rows))
            {
                continue;
            }

            cv::Vec3d previous_point_3d;
            cv::Vec3d current_point_3d;
            if (!reproject_to_3d(previous_frame_.features[i], previous_frame_.disparity, previous_point_3d) ||
                !reproject_to_3d(tracked_points[i], disparity, current_point_3d))
            {
                continue;
            }

            previous_points_3d.push_back(previous_point_3d);
            current_points_3d.push_back(current_point_3d);
            surviving_points.push_back(tracked_points[i]);
        }

        const auto motion = estimate_motion_ransac(previous_points_3d, current_points_3d);
        if (!motion.valid)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Stereo odom lost motion estimate; reseeding features.");
            reset_tracking(left_image, disparity, stamp);
            return;
        }

        const double translation_m = cv::norm(motion.translation);
        const double rotation_rad = rotation_angle_from_matrix(motion.rotation);
        if (translation_m > max_translation_per_frame_m_ || rotation_rad > max_rotation_per_frame_rad_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Stereo odom rejected motion spike (translation=%.3f m, rotation=%.3f rad).",
                translation_m,
                rotation_rad);
            reset_tracking(left_image, disparity, stamp);
            return;
        }

        publish_odometry(stamp, motion.rotation, motion.translation, dt_s);

        previous_frame_.stamp = stamp;
        previous_frame_.left_image = left_image.clone();
        previous_frame_.disparity = disparity.clone();
        previous_frame_.features.clear();
        previous_frame_.features.reserve(motion.inlier_indices.size());
        for (const int index : motion.inlier_indices)
        {
            previous_frame_.features.push_back(surviving_points[static_cast<size_t>(index)]);
        }

        if (static_cast<int>(previous_frame_.features.size()) < min_tracked_features_)
        {
            previous_frame_.features = detect_features(left_image, disparity);
        }
        has_previous_frame_ = !previous_frame_.features.empty();
    }

}  // namespace perseus_vision
