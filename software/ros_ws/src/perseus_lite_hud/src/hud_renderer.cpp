#include "perseus_lite_hud/hud_renderer.hpp"

namespace perseus_lite_hud
{

    void HudRenderer::add_element(std::shared_ptr<HudElementBase> element)
    {
        elements_.push_back(std::move(element));
    }

    void HudRenderer::render_all(cv::Mat& frame)
    {
        for (auto& el : elements_)
        {
            if (el->enabled() && el->is_ready())
            {
                el->render(frame);
            }
        }
    }

}  // namespace perseus_lite_hud
