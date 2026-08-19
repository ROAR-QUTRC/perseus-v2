#pragma once

/// @file voxel_downsampler.hpp
/// @brief Reduces Livox point cloud density, then Draco-compresses the surviving positions.

#include <cstdint>
#include <optional>
#include <perseus_interfaces/msg/compressed_point_cloud.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

namespace perseus_sensors
{
    /// @brief ROS 2 node that keeps at most one point per voxel cell, then Draco-compresses the
    ///        surviving points' x/y/z positions into a perseus_interfaces/CompressedPointCloud.
    ///
    /// Intensity is dropped; timestamp/tag/line (when present in the input cloud) travel
    /// alongside the compressed positions uncompressed, in the same per-point order, since
    /// FAST-LIO's motion compensation needs full timestamp precision.
    class VoxelDownsampler : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up the subscription
        ///        and publisher.
        /// @param options Node options, supplied by main().
        explicit VoxelDownsampler(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default topic that the raw point cloud is read from.
        static inline const std::string DEFAULT_INPUT_TOPIC = "/livox/lidar";
        /// @brief Default topic that the compressed point cloud is published on.
        static inline const std::string DEFAULT_OUTPUT_TOPIC = "/livox/lidar/downsampled";

        /// @brief Default voxel cell size, in metres.
        static constexpr double DEFAULT_VOXEL_SIZE_M = 0.1;
        /// @brief Default Draco position quantization, in bits.
        static constexpr int DEFAULT_QUANTIZATION_BITS = 14;

        /// @brief One surviving point's position and (if present in the input) side channels.
        struct point_t
        {
            float x{0.0f};
            float y{0.0f};
            float z{0.0f};
            double timestamp{0.0};
            uint8_t tag{0};
            uint8_t line{0};
        };

        /// @brief Byte offsets of each field this node reads, resolved once per incoming cloud.
        struct field_offsets_t
        {
            std::size_t x{0};
            std::size_t y{0};
            std::size_t z{0};
            std::optional<std::size_t> timestamp;
            std::optional<std::size_t> tag;
            std::optional<std::size_t> line;
        };

        /// @brief Declares every parameter and copies the topic names into the matching
        ///        out-parameters.
        /// @param input_topic_out Receives the resolved input point cloud topic.
        /// @param output_topic_out Receives the resolved compressed point cloud topic.
        void _load_parameters(std::string& input_topic_out, std::string& output_topic_out);

        /// @brief Downsamples and Draco-compresses an incoming cloud, then republishes it.
        /// @param msg Incoming raw point cloud.
        void _point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        /// @brief Resolves the byte offset of x/y/z (required) and timestamp/tag/line
        ///        (optional) fields in a cloud.
        /// @param msg Cloud to resolve field offsets against.
        /// @return The resolved offsets. x/y/z are always populated if this returns; callers
        ///         must separately confirm the cloud actually has float32 x/y/z fields.
        field_offsets_t _resolve_field_offsets(const sensor_msgs::msg::PointCloud2& msg) const;

        /// @brief Reads one point out of a cloud's raw byte buffer.
        /// @param msg Cloud to read from.
        /// @param offsets Field offsets, from _resolve_field_offsets.
        /// @param point_index Index of the point to read.
        static point_t _read_point(
            const sensor_msgs::msg::PointCloud2& msg,
            const field_offsets_t& offsets,
            std::size_t point_index);

        /// @brief Packs a voxel's integer coordinates into a single hashable key.
        static int64_t _voxel_key(int32_t vx, int32_t vy, int32_t vz);

        /// @brief Draco-encodes a set of points' positions, preserving their order.
        /// @param points Points whose x/y/z should be encoded.
        /// @return The encoded Draco buffer, or empty if encoding failed.
        std::vector<uint8_t> _encode_positions(const std::vector<point_t>& points) const;

        double _voxel_size_m{DEFAULT_VOXEL_SIZE_M};
        int _quantization_bits{DEFAULT_QUANTIZATION_BITS};

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscription;
        rclcpp::Publisher<perseus_interfaces::msg::CompressedPointCloud>::SharedPtr _publisher;
    };

}  // namespace perseus_sensors
