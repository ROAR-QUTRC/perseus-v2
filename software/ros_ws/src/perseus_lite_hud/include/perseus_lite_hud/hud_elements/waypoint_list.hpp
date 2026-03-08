#pragma once

#include <string>
#include <vector>

#include "perseus_lite_hud/hud_element_base.hpp"

namespace perseus_lite_hud
{

    struct WaypointEntry
    {
        std::string name;
        double x = 0.0;
        double y = 0.0;
    };

    struct WaypointListConfig
    {
        int width = 200;
        int row_height = 22;
        int max_visible = 8;
        double opacity = 0.7;
        int margin = 10;
    };

    class WaypointList : public HudElementBase
    {
    public:
        explicit WaypointList(const WaypointListConfig& cfg = {});

        void set_waypoints(const std::vector<WaypointEntry>& waypoints,
                           uint32_t current_index, bool active);
        void render(cv::Mat& frame) override;
        bool is_ready() const override;

    private:
        WaypointListConfig cfg_;
        std::vector<WaypointEntry> waypoints_;
        uint32_t current_index_ = 0;
        bool active_ = false;
        bool has_data_ = false;
    };

}  // namespace perseus_lite_hud
