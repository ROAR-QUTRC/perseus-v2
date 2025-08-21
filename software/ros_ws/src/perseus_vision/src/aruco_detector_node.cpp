#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector()
        : Node("aruco_detector")
    {
        // Camera parameters (replace with calibrated values)
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<  // NOTE: THIS IS CALIBRATED CAMERA MATRIX FOR SIMULATION!
                              530.4,
                          0, 320, !0, 530.4, 240,
                          0, 0, 1);
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        // Create ArUco dictionary and detector
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        detector_ = cv::aruco::ArucoDetector(dictionary_);

        // Create TF broadcaster and listener
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriber
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgbd_camera/image_raw", 10,
            std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));

        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/processed_aruco/image_raw", 10);

        RCLCPP_INFO(this->get_logger(), "Perseus' ArucoDetector node started.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image to OpenCV Mat
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Detect markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;  // List of detected marker corners (4 corners per marker)
        detector_.detectMarkers(frame, corners, ids);

        if (!ids.empty())
        {
            // Estimate pose for each marker
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.35, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i)
            {
                cv::aruco::drawDetectedMarkers(frame, corners, ids);
                cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.03);
                // tvecs[i][2] = 0; // Use X coordinate for display
                // Transform ArUco pose to base_link frame
                transformAndPublishMarker(msg->header, ids[i], rvecs[i], tvecs[i]);

                // Center coordinates
                cv::Point2f center(0, 0);
                for (const auto& pt : corners[i])
                    center += pt;
                center *= (1.0f / 4.0f);
                // tvecs[i][2] = 0; // Ignore Z for 2D display
                // tvecs[i][1] = 0; // Ignore Y for 2D display

                double distance = sqrt(tvecs[i][0] * tvecs[i][0] + tvecs[i][1] * tvecs[i][1] + tvecs[i][2] * tvecs[i][2]);
                // RCLCPP_INFO(this->get_logger(), "Marker ID: %d, Distance: %.2fm, Bearing: %.1f°",
                //             ids[i], distance, atan2(tvecs[i][0], tvecs[i][2]) * 180.0 / M_PI);
                double bearing = tvecs[i][0] != 0 ? atan2(tvecs[i][1], tvecs[i][0]) * 180.0 / M_PI : 0.0;

                RCLCPP_INFO(this->get_logger(), "X: %.2f m, Y: %.2f m, Z: %.2f m", tvecs[i][0], tvecs[i][1], tvecs[i][2]);
                RCLCPP_INFO(this->get_logger(), "Bearing: %.2f°", bearing);
                cv::putText(frame, "ID:" + std::to_string(ids[i]) + " D:" + std::to_string((int)(distance * 100)) + "cm",
                            corners[i][0] + cv::Point2f(0, -30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }
        }

        // Publish processed frame
        auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        pub_->publish(*processed_msg);
    }

    void transformAndPublishMarker(const std_msgs::msg::Header& header, int marker_id,
                                   const cv::Vec3d& rvec, const cv::Vec3d& tvec)
    {
        try
        {
            // Create pose in camera frame
            geometry_msgs::msg::PoseStamped marker_pose_camera;
            marker_pose_camera.header.stamp = header.stamp;
            marker_pose_camera.header.frame_id = "camera_link_optical";

            // Set position in camera frame
            marker_pose_camera.pose.position.x = tvec[2];   // Distance forward
            marker_pose_camera.pose.position.y = -tvec[0];  // Left/right
            marker_pose_camera.pose.position.z = -tvec[1];  // Up/down

            // Convert rotation vector to quaternion
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);
            tf2::Quaternion quat = rotationMatrixToQuaternion(rotation_matrix);

            marker_pose_camera.pose.orientation.x = quat.x();
            marker_pose_camera.pose.orientation.y = quat.y();
            marker_pose_camera.pose.orientation.z = quat.z();
            marker_pose_camera.pose.orientation.w = quat.w();

            // Transform to odom frame (fixed world frame)
            geometry_msgs::msg::PoseStamped marker_pose_odom;
            tf_buffer_->transform(marker_pose_camera, marker_pose_odom, "odom");

            // Publish transform in odom frame (fixed)
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = header.stamp;
            transform.header.frame_id = "odom";  // Changed to fixed frame
            transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

            transform.transform.translation.x = marker_pose_odom.pose.position.x;
            transform.transform.translation.y = marker_pose_odom.pose.position.y;
            transform.transform.translation.z = marker_pose_odom.pose.position.z;

            transform.transform.rotation = marker_pose_odom.pose.orientation;

            // Broadcast the transform in fixed frame
            tf_broadcaster_->sendTransform(transform);

            RCLCPP_INFO(this->get_logger(),
                        "ArUco %d in odom: x=%.2f, y=%.2f, z=%.2f",
                        marker_id, marker_pose_odom.pose.position.x,
                        marker_pose_odom.pose.position.y, marker_pose_odom.pose.position.z);
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform marker pose: %s", ex.what());
        }
    }

    tf2::Quaternion rotationMatrixToQuaternion(const cv::Mat& rotation_matrix)
    {
        double r11 = rotation_matrix.at<double>(0, 0);
        double r12 = rotation_matrix.at<double>(0, 1);
        double r13 = rotation_matrix.at<double>(0, 2);
        double r21 = rotation_matrix.at<double>(1, 0);
        double r22 = rotation_matrix.at<double>(1, 1);
        double r23 = rotation_matrix.at<double>(1, 2);
        double r31 = rotation_matrix.at<double>(2, 0);
        double r32 = rotation_matrix.at<double>(2, 1);
        double r33 = rotation_matrix.at<double>(2, 2);

        tf2::Quaternion quat;
        double trace = r11 + r22 + r33;
        if (trace > 0)
        {
            double s = sqrt(trace + 1.0) * 2;
            quat.setW(0.25 * s);
            quat.setX((r32 - r23) / s);
            quat.setY((r13 - r31) / s);
            quat.setZ((r21 - r12) / s);
        }
        else if ((r11 > r22) && (r11 > r33))
        {
            double s = sqrt(1.0 + r11 - r22 - r33) * 2;
            quat.setW((r32 - r23) / s);
            quat.setX(0.25 * s);
            quat.setY((r12 + r21) / s);
            quat.setZ((r13 + r31) / s);
        }
        else if (r22 > r33)
        {
            double s = sqrt(1.0 + r22 - r11 - r33) * 2;
            quat.setW((r13 - r31) / s);
            quat.setX((r12 + r21) / s);
            quat.setY(0.25 * s);
            quat.setZ((r23 + r32) / s);
        }
        else
        {
            double s = sqrt(1.0 + r33 - r11 - r22) * 2;
            quat.setW((r21 - r12) / s);
            quat.setX((r13 + r31) / s);
            quat.setY((r23 + r32) / s);
            quat.setZ(0.25 * s);
        }
        return quat;
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // ArUco
    cv::aruco::Dictionary dictionary_;
    cv::aruco::ArucoDetector detector_;

    // Camera intrinsics
    cv::Mat camera_matrix_, dist_coeffs_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}
