#pragma once

#include "perseus_lite_hud/hud_element_base.hpp"

namespace perseus_lite_hud
{

    struct VelocityGaugeConfig
    {
        int width = 60;
        int height = 160;
        double max_linear = 1.0;
        double max_angular = 2.0;
        double opacity = 0.7;
        int margin = 10;
    };

    class VelocityGauge : public HudElementBase
    {
    public:
        explicit VelocityGauge(const VelocityGaugeConfig& cfg = {});

        void render(cv::Mat& frame) override;
        bool is_ready() const override;

        void set_velocity(double linear, double angular);

    private:
        VelocityGaugeConfig cfg_;
        double linear_ = 0.0;
        double angular_ = 0.0;
        bool has_data_ = false;
    };

}  // namespace perseus_lite_hud
