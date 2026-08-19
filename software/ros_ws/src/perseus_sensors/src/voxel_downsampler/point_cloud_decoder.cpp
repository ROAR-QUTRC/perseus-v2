/// @file point_cloud_decoder.cpp
/// @brief Implementation of the CompressedPointCloud -> PointCloud2 decoder node.

#include "perseus_sensors/voxel_downsampler/point_cloud_decoder.hpp"

#include <cstring>
#include <draco/compression/decode.h>
#include <functional>
#include <sensor_msgs/msg/point_field.hpp>

namespace perseus_sensors
{
    namespace
    {
        /// @brief Queue depth used for the input and output topics.
        constexpr int QOS_DEPTH = 10;

        /// @brief Byte layout of one decoded point when timestamp/tag/line are present:
        ///        x,y,z (float32), timestamp (float64), tag, line (uint8).
        constexpr std::size_t X_OFFSET = 0;
        constexpr std::size_t Y_OFFSET = 4;
        constexpr std::size_t Z_OFFSET = 8;
        constexpr std::size_t TIMESTAMP_OFFSET = 12;
        constexpr std::size_t TAG_OFFSET = 20;
        constexpr std::size_t LINE_OFFSET = 21;
        constexpr std::size_t POINT_STEP_WITH_SIDE_CHANNELS = 22;

        /// @brief Byte layout of one decoded point when only x/y/z is available.
        constexpr std::size_t POINT_STEP_XYZ_ONLY = 12;

        /// @brief Builds the x/y/z-only field layout for the output cloud.
        std::vector<sensor_msgs::msg::PointField> make_xyz_fields()
        {
            std::vector<sensor_msgs::msg::PointField> fields(3);
            fields[0].name = "x";
            fields[0].offset = X_OFFSET;
            fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            fields[0].count = 1;
            fields[1].name = "y";
            fields[1].offset = Y_OFFSET;
            fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            fields[1].count = 1;
            fields[2].name = "z";
            fields[2].offset = Z_OFFSET;
            fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            fields[2].count = 1;
            return fields;
        }

        /// @brief Builds the x/y/z/timestamp/tag/line field layout for the output cloud.
        std::vector<sensor_msgs::msg::PointField> make_full_fields()
        {
            std::vector<sensor_msgs::msg::PointField> fields = make_xyz_fields();
            fields.resize(6);
            fields[3].name = "timestamp";
            fields[3].offset = TIMESTAMP_OFFSET;
            fields[3].datatype = sensor_msgs::msg::PointField::FLOAT64;
            fields[3].count = 1;
            fields[4].name = "tag";
            fields[4].offset = TAG_OFFSET;
            fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
            fields[4].count = 1;
            fields[5].name = "line";
            fields[5].offset = LINE_OFFSET;
            fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
            fields[5].count = 1;
            return fields;
        }
    }

    PointCloudDecoder::PointCloudDecoder(const rclcpp::NodeOptions& options)
        : rclcpp::Node("point_cloud_decoder", options)
    {
        const std::string input_topic = this->declare_parameter("input_topic", DEFAULT_INPUT_TOPIC);
        const std::string output_topic =
            this->declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC);

        _compressed_subscription =
            this->create_subscription<perseus_interfaces::msg::CompressedPointCloud>(
                input_topic,
                QOS_DEPTH,
                std::bind(
                    &PointCloudDecoder::_compressed_point_cloud_callback, this,
                    std::placeholders::_1));

        _points_publisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, QOS_DEPTH);

        RCLCPP_INFO(
            this->get_logger(),
            "Point cloud decoder: %s -> %s",
            input_topic.c_str(),
            output_topic.c_str());
    }

    void PointCloudDecoder::_compressed_point_cloud_callback(
        const perseus_interfaces::msg::CompressedPointCloud::SharedPtr msg)
    {
        draco::DecoderBuffer buffer;
        buffer.Init(reinterpret_cast<const char*>(msg->draco_data.data()), msg->draco_data.size());

        draco::Decoder decoder;
        draco::StatusOr<std::unique_ptr<draco::PointCloud>> decode_status =
            decoder.DecodePointCloudFromBuffer(&buffer);
        if (!decode_status.ok())
        {
            RCLCPP_ERROR(
                this->get_logger(), "Draco decoding failed: %s", decode_status.status().error_msg());
            return;
        }
        const std::unique_ptr<draco::PointCloud> point_cloud = std::move(decode_status).value();

        const draco::PointAttribute* position_attribute =
            point_cloud->GetNamedAttribute(draco::GeometryAttribute::POSITION);
        if (position_attribute == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Decoded point cloud has no POSITION attribute.");
            return;
        }

        const std::size_t point_count = static_cast<std::size_t>(point_cloud->num_points());

        // timestamp/tag/line travel uncompressed, in the same per-point order as the
        // Draco-encoded positions (see voxel_downsampler.cpp). They may be absent if the
        // source cloud didn't have those fields (e.g. simulated Livox data), in which case we
        // publish x/y/z only rather than treating it as an error.
        const bool has_side_channels = point_count == msg->timestamps.size() &&
                                       point_count == msg->tags.size() &&
                                       point_count == msg->lines.size() && point_count > 0;

        sensor_msgs::msg::PointCloud2 output;
        output.header = msg->header;
        output.height = 1;
        output.width = static_cast<uint32_t>(point_count);
        output.fields = has_side_channels ? make_full_fields() : make_xyz_fields();
        output.is_bigendian = false;
        output.point_step =
            has_side_channels ? POINT_STEP_WITH_SIDE_CHANNELS : POINT_STEP_XYZ_ONLY;
        output.row_step = output.width * output.point_step;
        output.is_dense = true;
        output.data.resize(static_cast<std::size_t>(output.row_step));

        for (std::size_t i = 0; i < point_count; ++i)
        {
            float position[3] = {0.0f, 0.0f, 0.0f};
            position_attribute->GetMappedValue(draco::PointIndex(static_cast<uint32_t>(i)), position);

            const std::size_t byte_offset = i * output.point_step;
            std::memcpy(&output.data[byte_offset + X_OFFSET], &position[0], sizeof(float));
            std::memcpy(&output.data[byte_offset + Y_OFFSET], &position[1], sizeof(float));
            std::memcpy(&output.data[byte_offset + Z_OFFSET], &position[2], sizeof(float));

            if (has_side_channels)
            {
                const double timestamp = msg->timestamps[i];
                const uint8_t tag = msg->tags[i];
                const uint8_t line = msg->lines[i];
                std::memcpy(&output.data[byte_offset + TIMESTAMP_OFFSET], &timestamp, sizeof(double));
                output.data[byte_offset + TAG_OFFSET] = tag;
                output.data[byte_offset + LINE_OFFSET] = line;
            }
        }

        _points_publisher->publish(output);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Decoded %zu points from %zu Draco bytes%s",
            point_count,
            msg->draco_data.size(),
            has_side_channels ? "" : " (no timestamp/tag/line)");
    }

}  // namespace perseus_sensors
