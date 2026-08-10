/// @file dust_filter.cpp
/// @brief Implementation of the point-cloud declutter node.

#include "perseus_sensors/dust_filter/dust_filter.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <optional>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace perseus_sensors
{
    namespace
    {
        /// @brief Queue depth used for the input and output point cloud topics.
        constexpr int QOS_DEPTH = 10;

        /// @brief Bits of voxel-grid range packed per axis into a hash key.
        ///
        /// 2^20 cells either side of the origin, which even at a 1 cm search radius covers a
        /// ~10 km cube -- far beyond any range this robot operates at.
        constexpr int32_t VOXEL_KEY_BITS_PER_AXIS = 21;
        constexpr int64_t VOXEL_KEY_AXIS_MASK = (int64_t(1) << VOXEL_KEY_BITS_PER_AXIS) - 1;
        constexpr int32_t VOXEL_KEY_AXIS_OFFSET = 1 << (VOXEL_KEY_BITS_PER_AXIS - 1);
    }

    DustFilter::DustFilter(const rclcpp::NodeOptions& options)
        : rclcpp::Node("dust_filter", options)
    {
        std::string input_topic;
        std::string output_topic;
        _load_parameters(input_topic, output_topic);

        _point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic,
            QOS_DEPTH,
            std::bind(&DustFilter::_point_cloud_callback, this, std::placeholders::_1));

        _filtered_publisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, QOS_DEPTH);
    }

    void DustFilter::_load_parameters(std::string& input_topic_out, std::string& output_topic_out)
    {
        input_topic_out = this->declare_parameter("input_topic", DEFAULT_INPUT_TOPIC);
        output_topic_out = this->declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC);
        _search_radius_m = static_cast<float>(
            this->declare_parameter("search_radius_m", static_cast<double>(DEFAULT_SEARCH_RADIUS_M)));
        _min_neighbors = static_cast<std::size_t>(
            this->declare_parameter("min_neighbors", static_cast<int>(DEFAULT_MIN_NEIGHBORS)));
    }

    void DustFilter::_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const auto callback_start = std::chrono::steady_clock::now();

        const std::size_t point_count = static_cast<std::size_t>(msg->width) * msg->height;
        const std::vector<indexed_point_t> points = _extract_points(*msg);
        if (points.empty() && point_count > 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Point cloud has no float32 x/y/z fields; republishing it unfiltered.");
            _filtered_publisher->publish(*msg);
            return;
        }

        const std::unordered_map<int64_t, std::vector<uint32_t>> voxels = _bucket_into_voxels(points);

        sensor_msgs::msg::PointCloud2 output;
        output.header = msg->header;
        output.height = 1;
        output.fields = msg->fields;
        output.is_bigendian = msg->is_bigendian;
        output.point_step = msg->point_step;
        output.is_dense = true;
        output.data.reserve(msg->data.size());

        std::size_t surviving_count = 0;
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            if (_count_neighbors_within_radius(points, voxels, i) < _min_neighbors)
            {
                continue;
            }

            const std::size_t byte_offset = points[i].byte_offset;
            output.data.insert(
                output.data.end(),
                msg->data.begin() + byte_offset,
                msg->data.begin() + byte_offset + msg->point_step);
            ++surviving_count;
        }

        output.width = static_cast<uint32_t>(surviving_count);
        output.row_step = output.width * output.point_step;

        _filtered_publisher->publish(output);

        const auto callback_elapsed = std::chrono::steady_clock::now() - callback_start;
        const auto callback_elapsed_ms =
            std::chrono::duration<double, std::milli>(callback_elapsed).count();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Filtered %zu/%zu points in %.1f ms",
            surviving_count,
            point_count,
            callback_elapsed_ms);
    }

    std::vector<DustFilter::indexed_point_t> DustFilter::_extract_points(
        const sensor_msgs::msg::PointCloud2& msg) const
    {
        std::optional<std::size_t> x_offset;
        std::optional<std::size_t> y_offset;
        std::optional<std::size_t> z_offset;
        for (const auto& field : msg.fields)
        {
            if (field.datatype != sensor_msgs::msg::PointField::FLOAT32)
            {
                continue;
            }
            if (field.name == "x")
            {
                x_offset = field.offset;
            }
            else if (field.name == "y")
            {
                y_offset = field.offset;
            }
            else if (field.name == "z")
            {
                z_offset = field.offset;
            }
        }
        if (!x_offset.has_value() || !y_offset.has_value() || !z_offset.has_value())
        {
            return {};
        }

        const std::size_t point_count = static_cast<std::size_t>(msg.width) * msg.height;
        std::vector<indexed_point_t> points;
        points.reserve(point_count);
        for (std::size_t i = 0; i < point_count; ++i)
        {
            indexed_point_t point;
            point.byte_offset = i * msg.point_step;
            std::memcpy(&point.x, &msg.data[point.byte_offset + x_offset.value()], sizeof(float));
            std::memcpy(&point.y, &msg.data[point.byte_offset + y_offset.value()], sizeof(float));
            std::memcpy(&point.z, &msg.data[point.byte_offset + z_offset.value()], sizeof(float));
            points.push_back(point);
        }
        return points;
    }

    std::unordered_map<int64_t, std::vector<uint32_t>> DustFilter::_bucket_into_voxels(
        const std::vector<indexed_point_t>& points) const
    {
        std::unordered_map<int64_t, std::vector<uint32_t>> voxels;
        voxels.reserve(points.size());
        for (uint32_t i = 0; i < points.size(); ++i)
        {
            const indexed_point_t& point = points[i];
            const int32_t ix = static_cast<int32_t>(std::floor(point.x / _search_radius_m));
            const int32_t it = static_cast<int32_t>(std::floor(point.y / _search_radius_m));
            const int32_t is = static_cast<int32_t>(std::floor(point.z / _search_radius_m));
            voxels[_voxel_key(ix, it, is)].push_back(i);
        }
        return voxels;
    }

    std::size_t DustFilter::_count_neighbors_within_radius(
        const std::vector<indexed_point_t>& points,
        const std::unordered_map<int64_t, std::vector<uint32_t>>& voxels,
        std::size_t query_index) const
    {
        const indexed_point_t& query = points[query_index];
        const int32_t ix = static_cast<int32_t>(std::floor(query.x / _search_radius_m));
        const int32_t it = static_cast<int32_t>(std::floor(query.y / _search_radius_m));
        const int32_t is = static_cast<int32_t>(std::floor(query.z / _search_radius_m));
        const float radius_squared = _search_radius_m * _search_radius_m;

        std::size_t neighbor_count = 0;
        for (int32_t dx = -1; dx <= 1 && neighbor_count < _min_neighbors; ++dx)
        {
            for (int32_t dy = -1; dy <= 1 && neighbor_count < _min_neighbors; ++dy)
            {
                for (int32_t dz = -1; dz <= 1 && neighbor_count < _min_neighbors; ++dz)
                {
                    const auto bucket = voxels.find(_voxel_key(ix + dx, it + dy, is + dz));
                    if (bucket != voxels.end())
                    {
                        neighbor_count += _count_matches_in_bucket(
                            bucket->second,
                            points,
                            query,
                            query_index,
                            radius_squared,
                            _min_neighbors - neighbor_count);
                    }
                }
            }
        }
        return neighbor_count;
    }

    std::size_t DustFilter::_count_matches_in_bucket(
        const std::vector<uint32_t>& bucket_indices,
        const std::vector<indexed_point_t>& points,
        const indexed_point_t& query,
        std::size_t query_index,
        float radius_squared,
        std::size_t max_matches_needed) const
    {
        std::size_t match_count = 0;
        for (const uint32_t candidate_index : bucket_indices)
        {
            if (match_count >= max_matches_needed)
            {
                break;
            }
            if (candidate_index == query_index)
            {
                continue;
            }
            const indexed_point_t& candidate = points[candidate_index];
            const float dx = candidate.x - query.x;
            const float dy = candidate.y - query.y;
            const float dz = candidate.z - query.z;
            if (dx * dx + dy * dy + dz * dz <= radius_squared)
            {
                ++match_count;
            }
        }
        return match_count;
    }

    int64_t DustFilter::_voxel_key(int32_t ix, int32_t it, int32_t is)
    {
        const int64_t x_component = (static_cast<int64_t>(ix) + VOXEL_KEY_AXIS_OFFSET) & VOXEL_KEY_AXIS_MASK;
        const int64_t y_component = (static_cast<int64_t>(it) + VOXEL_KEY_AXIS_OFFSET) & VOXEL_KEY_AXIS_MASK;
        const int64_t z_component = (static_cast<int64_t>(is) + VOXEL_KEY_AXIS_OFFSET) & VOXEL_KEY_AXIS_MASK;
        return x_component | (y_component << VOXEL_KEY_BITS_PER_AXIS) |
               (z_component << (2 * VOXEL_KEY_BITS_PER_AXIS));
    }

}  // namespace perseus_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(perseus_sensors::DustFilter)
