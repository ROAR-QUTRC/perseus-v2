#pragma once

#include "perseus_lite_hud/hud_element_base.hpp"

namespace perseus_lite_hud
{

    struct WaypointListConfig
    {
        int width = 200;
        int height = 300;
        double opacity = 0.7;
    };

    class WaypointList : public HudElementBase
    {
    public:
        explicit WaypointList(const WaypointListConfig& cfg = {});

        void render(cv::Mat& frame) override;
        bool is_ready() const override;

    private:
        WaypointListConfig cfg_;
    };

}  // namespace perseus_lite_hud
