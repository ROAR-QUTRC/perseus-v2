#pragma once

/// @file point_cloud_decoder.hpp
/// @brief Draco-decodes a CompressedPointCloud back into a sensor_msgs/PointCloud2, for local
///        visualization (e.g. in RViz) without spending bandwidth on the wire.

#include <perseus_interfaces/msg/compressed_point_cloud.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace perseus_sensors
{
    /// @brief ROS 2 node that decodes a perseus_interfaces/CompressedPointCloud back into a
    ///        sensor_msgs/PointCloud2.
    ///
    /// Publishes x/y/z always; timestamp/tag/line are added only when the source cloud had
    /// them (its timestamps/tags/lines arrays are all the same length as the decoded point
    /// count), since simulated Livox data typically lacks those fields.
    ///
    /// Intended to run only where a decoded cloud is actually needed, such as a visualization
    /// workstation: the compressed representation is what should cross a bandwidth-constrained
    /// link (e.g. rover to base station), and decoding it back to PointCloud2 undoes that
    /// saving, so this node should sit downstream of that link, not upstream of it.
    class PointCloudDecoder : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up the subscription
        ///        and publisher.
        /// @param options Node options, supplied by main().
        explicit PointCloudDecoder(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default topic that the compressed point cloud is read from.
        static inline const std::string DEFAULT_INPUT_TOPIC = "/livox/lidar/downsampled";
        /// @brief Default topic that the decoded point cloud is published on.
        static inline const std::string DEFAULT_OUTPUT_TOPIC = "/livox/lidar/downsampled/points";

        /// @brief Decodes an incoming compressed cloud and republishes it as PointCloud2.
        /// @param msg Incoming compressed point cloud.
        void _compressed_point_cloud_callback(
            const perseus_interfaces::msg::CompressedPointCloud::SharedPtr msg);

        rclcpp::Subscription<perseus_interfaces::msg::CompressedPointCloud>::SharedPtr
            _compressed_subscription;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _points_publisher;
    };

}  // namespace perseus_sensors
