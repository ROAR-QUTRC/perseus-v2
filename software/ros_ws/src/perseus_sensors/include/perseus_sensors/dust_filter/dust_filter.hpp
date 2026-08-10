#pragma once

/// @file dust_filter.hpp
/// @brief Removes locally sparse point-cloud returns caused by airborne dust or sand.

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace perseus_sensors
{
    /// @brief ROS 2 node that declutters a Livox point cloud before it reaches FAST-LIO or
    ///        the obstacle costmap.
    ///
    /// Applies radius outlier removal: a point survives only if at least `min_neighbors`
    /// other points fall within `search_radius_m` of it. Airborne dust and sand thrown up by
    /// the robot's own wheels scatters light diffusely and returns as isolated points with no
    /// coherent surface behind them, whereas a solid obstacle returns a dense cluster at the
    /// sensor's angular resolution -- so this separates the two structurally rather than by
    /// reflectivity, which real low-reflectivity hazards can share with dust.
    ///
    /// Implemented directly against the raw PointCloud2 buffer with a uniform voxel grid
    /// (cell size equal to the search radius) rather than a PCL-based kd-tree, so that each
    /// point only checks its 27 neighbouring cells instead of every other point, and so that
    /// fields the Livox driver defines beyond x/y/z/intensity -- tag, line, timestamp, all of
    /// which FAST-LIO's own preprocessing reads -- pass through untouched instead of being
    /// dropped by a conversion to a plain XYZI point type.
    ///
    /// This is a single-frame, low-latency pass deliberately kept simple: its job is to keep
    /// dust out of FAST-LIO's persistent ikd-tree map (built from every point in a scan
    /// regardless of whether that point influenced the scan's pose estimate -- see
    /// MapIncremental in FAST-LIO's laserMapping.cpp), not to catch every dust return outright.
    /// A fixed search radius is also less selective close to the sensor, where its angular
    /// resolution already packs points tighter regardless of what they hit -- if dust near the
    /// robot is still getting through, that is the parameter to revisit before reaching for a
    /// heavier, multi-frame technique.
    class DustFilter : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up the subscription
        ///        and publisher.
        /// @param options Node options, supplied by main().
        explicit DustFilter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default topic that the raw point cloud is read from.
        static inline const std::string DEFAULT_INPUT_TOPIC = "/livox/lidar";
        /// @brief Default topic that the filtered point cloud is published on.
        static inline const std::string DEFAULT_OUTPUT_TOPIC = "/livox/lidar/filtered";

        /// @brief Default radius searched around each point for neighbours, in metres.
        static constexpr float DEFAULT_SEARCH_RADIUS_M = 0.15f;
        /// @brief Default neighbour count within the search radius required to keep a point.
        static constexpr std::size_t DEFAULT_MIN_NEIGHBORS = 5;

        /// @brief A point's position and where it starts in the source cloud's byte buffer.
        struct indexed_point_t
        {
            float x{0.0f};
            float y{0.0f};
            float z{0.0f};
            std::size_t byte_offset{0};
        };

        /// @brief Declares every parameter and copies the topic names into the matching
        ///        out-parameters.
        /// @param input_topic_out Receives the resolved input point cloud topic.
        /// @param output_topic_out Receives the resolved filtered point cloud topic.
        void _load_parameters(std::string& input_topic_out, std::string& output_topic_out);

        /// @brief Filters an incoming cloud and republishes the result.
        /// @param msg Incoming raw point cloud.
        void _point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        /// @brief Reads every point's position out of a cloud, keyed to its byte offset.
        /// @param msg Cloud to read.
        /// @return One entry per point, in the cloud's original order. Empty if the cloud has
        ///         no float32 x/y/z fields, regardless of how many points it claims to have.
        std::vector<indexed_point_t> _extract_points(const sensor_msgs::msg::PointCloud2& msg) const;

        /// @brief Buckets points into a uniform grid so radius queries only check nearby cells.
        /// @param points Points to bucket, indexed by their position in this vector.
        /// @return Map from a packed voxel coordinate to the indices of points inside it.
        std::unordered_map<int64_t, std::vector<uint32_t>> _bucket_into_voxels(
            const std::vector<indexed_point_t>& points) const;

        /// @brief Counts how many other points fall within the search radius of one point.
        /// @param points All points, indexed the same way as @p voxels.
        /// @param voxels Voxel bucketing of @p points, from _bucket_into_voxels.
        /// @param query_index Index into @p points of the point to search around.
        /// @return The neighbour count, capped at `min_neighbors` -- counting stops once a
        ///         point is confirmed to survive, since the exact count above that is unused.
        std::size_t _count_neighbors_within_radius(
            const std::vector<indexed_point_t>& points,
            const std::unordered_map<int64_t, std::vector<uint32_t>>& voxels,
            std::size_t query_index) const;

        /// @brief Counts points in one voxel bucket within the search radius of a query point.
        /// @param bucket_indices Indices (into @p points) of the points in this bucket.
        /// @param points All points, indexed the same way as @p bucket_indices.
        /// @param query Point to measure distance from.
        /// @param query_index Index of @p query into @p points, excluded from its own count.
        /// @param radius_squared Search radius, squared, to avoid a sqrt per distance check.
        /// @param max_matches_needed Stops scanning the bucket once this many matches are
        ///        found -- a dense near-field bucket can hold far more points than are needed
        ///        to decide a point survives, and every point sharing that bucket would
        ///        otherwise re-scan all of it.
        /// @return The number of matching points in this bucket, capped at @p max_matches_needed.
        std::size_t _count_matches_in_bucket(
            const std::vector<uint32_t>& bucket_indices,
            const std::vector<indexed_point_t>& points,
            const indexed_point_t& query,
            std::size_t query_index,
            float radius_squared,
            std::size_t max_matches_needed) const;

        /// @brief Packs a voxel's integer coordinates into a single hashable key.
        /// @param ix Voxel coordinate along x, in units of the search radius.
        /// @param it Voxel coordinate along y, in units of the search radius.
        /// @param is Voxel coordinate along z, in units of the search radius.
        static int64_t _voxel_key(int32_t ix, int32_t it, int32_t is);

        float _search_radius_m{DEFAULT_SEARCH_RADIUS_M};
        std::size_t _min_neighbors{DEFAULT_MIN_NEIGHBORS};

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscription;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _filtered_publisher;
    };

}  // namespace perseus_sensors
