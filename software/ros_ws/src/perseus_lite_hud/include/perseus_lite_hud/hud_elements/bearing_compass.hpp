#pragma once

#include "perseus_lite_hud/hud_element_base.hpp"

namespace perseus_lite_hud
{

    struct BearingCompassConfig
    {
        int width = 400;
        int height = 40;
        int degrees_visible = 120;
        double opacity = 0.7;
    };

    class BearingCompass : public HudElementBase
    {
    public:
        explicit BearingCompass(const BearingCompassConfig& cfg = {});

        void render(cv::Mat& frame) override;
        bool is_ready() const override;

        void set_heading(double heading_deg);

    private:
        void _draw_graduation(cv::Mat& strip, double center_deg);
        void _draw_indicator(cv::Mat& frame, const cv::Point& center_top);

        BearingCompassConfig cfg_;
        double heading_deg_ = 0.0;
        bool has_data_ = false;
    };

}  // namespace perseus_lite_hud
