#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "perseus_lite_hud/hud_element_base.hpp"

namespace perseus_lite_hud
{

    struct PipMapConfig
    {
        int size = 180;
        double radius_m = 5.0;
        double opacity = 0.8;
        int margin = 10;
    };

    class PipMap : public HudElementBase
    {
    public:
        explicit PipMap(const PipMapConfig& cfg = {});

        void render(cv::Mat& frame) override;
        bool is_ready() const override;

        void set_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);
        void set_robot_pose(double x, double y, double yaw);

    private:
        void _rebuild_map_image();

        PipMapConfig cfg_;

        nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
        cv::Mat map_image_;  // Full map as BGR
        bool map_image_dirty_ = true;

        double robot_x_ = 0.0;
        double robot_y_ = 0.0;
        double robot_yaw_ = 0.0;
        bool has_map_ = false;
        bool has_pose_ = false;
    };

}  // namespace perseus_lite_hud
