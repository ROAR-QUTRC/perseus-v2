#pragma once

#include <memory>
#include <vector>

#include "perseus_lite_hud/hud_element_base.hpp"

namespace perseus_lite_hud
{

    class HudRenderer
    {
    public:
        HudRenderer() = default;

        void add_element(std::shared_ptr<HudElementBase> element);
        void render_all(cv::Mat& frame);

    private:
        std::vector<std::shared_ptr<HudElementBase>> elements_;
    };

}  // namespace perseus_lite_hud
