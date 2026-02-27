#include "network_recovery/heatmap.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace network_recovery
{

    // -- HeatmapCell --

    void HeatmapCell::update(int rssi, double now)
    {
        if (rssi_count == 0)
        {
            rssi_sum = rssi;
            rssi_count = 1;
            rssi_min = rssi;
            rssi_max = rssi;
        }
        else
        {
            rssi_sum += rssi;
            rssi_count++;
            rssi_min = std::min(rssi_min, rssi);
            rssi_max = std::max(rssi_max, rssi);
        }
        last_updated = now;
    }

    double HeatmapCell::rssi_avg() const
    {
        if (rssi_count == 0)
            return -100.0;
        return static_cast<double>(rssi_sum) / rssi_count;
    }

    // -- HeatmapGrid --

    HeatmapGrid::HeatmapGrid(const Config& cfg)
        : cfg_(cfg)
    {
    }

    GridKey HeatmapGrid::to_grid(double x, double y) const
    {
        return {static_cast<int>(std::floor(x / cfg_.cell_size_m)),
                static_cast<int>(std::floor(y / cfg_.cell_size_m))};
    }

    std::pair<double, double> HeatmapGrid::to_world(int gx, int gy) const
    {
        return {(gx + 0.5) * cfg_.cell_size_m, (gy + 0.5) * cfg_.cell_size_m};
    }

    void HeatmapGrid::record(double x, double y, int rssi)
    {
        auto key = to_grid(x, y);
        auto now = std::chrono::duration<double>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count();
        cells_[key].update(rssi, now);
    }

    std::optional<geometry_msgs::msg::PoseStamped>
    HeatmapGrid::find_nearest_good_signal(double current_x, double current_y,
                                          int rssi_threshold) const
    {
        int threshold =
            (rssi_threshold != 0) ? rssi_threshold : cfg_.good_signal_threshold;
        double best_dist_sq = std::numeric_limits<double>::infinity();
        std::optional<std::pair<double, double>> best_pos;

        for (const auto& [key, cell] : cells_)
        {
            if (cell.rssi_avg() >= threshold)
            {
                auto [wx, wy] = to_world(key.first, key.second);
                double dx = wx - current_x;
                double dy = wy - current_y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq < best_dist_sq)
                {
                    best_dist_sq = dist_sq;
                    best_pos = {wx, wy};
                }
            }
        }

        if (!best_pos)
            return std::nullopt;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = cfg_.map_frame;
        pose.pose.position.x = best_pos->first;
        pose.pose.position.y = best_pos->second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    std::tuple<float, float, float> HeatmapGrid::rssi_to_colour(double rssi_avg)
    {
        if (rssi_avg >= -50)
            return {0.0f, 1.0f, 0.0f};  // Green
        if (rssi_avg >= -60)
            return {0.5f, 1.0f, 0.0f};  // Yellow-green
        if (rssi_avg >= -70)
            return {1.0f, 1.0f, 0.0f};  // Yellow
        if (rssi_avg >= -80)
            return {1.0f, 0.5f, 0.0f};  // Orange
        if (rssi_avg >= -90)
            return {1.0f, 0.0f, 0.0f};  // Red
        return {0.5f, 0.0f, 0.0f};      // Dark red
    }

    visualization_msgs::msg::MarkerArray HeatmapGrid::to_marker_array(
        const builtin_interfaces::msg::Time& stamp) const
    {
        visualization_msgs::msg::MarkerArray ma;

        if (cells_.empty())
        {
            visualization_msgs::msg::Marker del;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            ma.markers.push_back(del);
            return ma;
        }

        int idx = 0;
        for (const auto& [key, cell] : cells_)
        {
            auto [wx, wy] = to_world(key.first, key.second);
            auto [r, g, b] = rssi_to_colour(cell.rssi_avg());

            visualization_msgs::msg::Marker m;
            m.header.frame_id = cfg_.map_frame;
            m.header.stamp = stamp;
            m.ns = "wifi_heatmap";
            m.id = idx++;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = wx;
            m.pose.position.y = wy;
            m.pose.position.z = cfg_.marker_z;
            m.pose.orientation.w = 1.0;
            m.scale.x = cfg_.cell_size_m;
            m.scale.y = cfg_.cell_size_m;
            m.scale.z = cfg_.marker_height;
            m.color.r = r;
            m.color.g = g;
            m.color.b = b;
            m.color.a = static_cast<float>(cfg_.marker_alpha);

            ma.markers.push_back(std::move(m));
        }

        return ma;
    }

    void HeatmapGrid::save_to_disk(const std::string& save_directory,
                                   rclcpp::Logger logger) const
    {
        if (cells_.empty())
        {
            RCLCPP_INFO(logger, "Heatmap empty, skipping save");
            return;
        }

        namespace fs = std::filesystem;
        std::string save_dir = save_directory;
        // Expand ~ manually
        if (!save_dir.empty() && save_dir[0] == '~')
        {
            const char* home = std::getenv("HOME");
            if (home)
                save_dir = std::string(home) + save_dir.substr(1);
        }

        try
        {
            fs::create_directories(save_dir);
        }
        catch (const fs::filesystem_error& e)
        {
            RCLCPP_ERROR(logger, "Failed to create heatmap directory %s: %s",
                         save_dir.c_str(), e.what());
            return;
        }

        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        char ts[32];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t));
        std::string filepath = save_dir + "/heatmap_" + ts + ".json";

        std::ofstream ofs(filepath);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR(logger, "Failed to open %s for writing", filepath.c_str());
            return;
        }

        ofs << std::fixed << std::setprecision(3);
        ofs << "{\n";
        ofs << "  \"map_frame\": \"" << cfg_.map_frame << "\",\n";
        ofs << "  \"cell_size_m\": " << cfg_.cell_size_m << ",\n";
        ofs << "  \"total_cells\": " << cells_.size() << ",\n";
        ofs << "  \"cells\": [\n";

        bool first = true;
        for (const auto& [key, cell] : cells_)
        {
            auto [wx, wy] = to_world(key.first, key.second);
            if (!first)
                ofs << ",\n";
            first = false;
            ofs << "    {\"x\": " << wx
                << ", \"y\": " << wy
                << ", \"rssi_avg\": " << std::setprecision(1) << cell.rssi_avg()
                << std::setprecision(3)
                << ", \"rssi_min\": " << cell.rssi_min
                << ", \"rssi_max\": " << cell.rssi_max
                << ", \"count\": " << cell.rssi_count << "}";
        }

        ofs << "\n  ]\n}\n";
        ofs.close();

        RCLCPP_INFO(logger, "Heatmap saved to %s (%zu cells)", filepath.c_str(),
                    cells_.size());
    }

}  // namespace network_recovery
