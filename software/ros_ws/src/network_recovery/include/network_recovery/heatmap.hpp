#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <string>
#include <unordered_map>
#include <visualization_msgs/msg/marker_array.hpp>

namespace network_recovery
{

    struct HeatmapCell
    {
        long rssi_sum{0};
        int rssi_count{0};
        int rssi_min{0};
        int rssi_max{-200};
        double last_updated{0.0};

        void update(int rssi, double now);
        double rssi_avg() const;
    };

    /// Hash for grid coordinates.
    struct GridKeyHash
    {
        std::size_t operator()(const std::pair<int, int>& k) const
        {
            auto h1 = std::hash<int>{}(k.first);
            auto h2 = std::hash<int>{}(k.second);
            return h1 ^ (h2 << 16);
        }
    };

    using GridKey = std::pair<int, int>;

    /// Discretized grid that stores WiFi signal strength per cell.
    class HeatmapGrid
    {
    public:
        struct Config
        {
            double cell_size_m{0.5};
            double marker_height{0.1};
            double marker_z{0.05};
            double marker_alpha{0.6};
            int good_signal_threshold{-55};
            std::string map_frame{"map"};
        };

        explicit HeatmapGrid(const Config& cfg);

        void record(double x, double y, int rssi);

        /// Find the nearest cell with average RSSI above threshold.
        std::optional<geometry_msgs::msg::PoseStamped> find_nearest_good_signal(
            double current_x, double current_y,
            int rssi_threshold = 0) const;

        visualization_msgs::msg::MarkerArray to_marker_array(
            const builtin_interfaces::msg::Time& stamp) const;

        void save_to_disk(const std::string& save_directory,
                          rclcpp::Logger logger) const;

        bool empty() const { return cells_.empty(); }

    private:
        GridKey to_grid(double x, double y) const;
        std::pair<double, double> to_world(int gx, int gy) const;
        static std::tuple<float, float, float> rssi_to_colour(double rssi_avg);

        Config cfg_;
        std::unordered_map<GridKey, HeatmapCell, GridKeyHash> cells_;
    };

}  // namespace network_recovery
