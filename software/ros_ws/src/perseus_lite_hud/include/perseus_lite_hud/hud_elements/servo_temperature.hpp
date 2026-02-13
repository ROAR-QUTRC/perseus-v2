#pragma once

#include <array>
#include <chrono>

#include "perseus_lite_hud/hud_element_base.hpp"

namespace perseus_lite_hud
{

    struct ServoTemperatureConfig
    {
        int width = 200;
        int height = 180;
        double warning_temp = 50.0;
        double danger_temp = 65.0;
        double max_temp = 70.0;
        double min_temp = 25.0;
        double opacity = 0.75;
        int margin = 10;
        double flash_rate_hz = 3.0;
    };

    class ServoTemperature : public HudElementBase
    {
    public:
        explicit ServoTemperature(const ServoTemperatureConfig& cfg = {});

        void render(cv::Mat& frame) override;
        bool is_ready() const override;

        // temps: [FL, FR, RL, RR]
        void set_temperatures(const std::array<double, 4>& temps);

    private:
        cv::Scalar _color_for_temp(double temp_c) const;
        bool _should_flash(double temp_c) const;
        void _draw_corner_brackets(cv::Mat& panel) const;
        void _draw_chassis(cv::Mat& panel, int cx, int cy, int cw, int ch) const;
        void _draw_wheel(cv::Mat& panel, int x, int y, int w, int h, double temp,
                         const std::string& label) const;
        void _draw_scale_bar(cv::Mat& panel, int x, int y, int w, int h) const;

        ServoTemperatureConfig cfg_;
        std::array<double, 4> temps_{};
        bool has_data_ = false;
        std::chrono::steady_clock::time_point start_time_;
    };

}  // namespace perseus_lite_hud
