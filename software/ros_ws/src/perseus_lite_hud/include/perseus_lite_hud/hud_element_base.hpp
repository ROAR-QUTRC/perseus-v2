#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

namespace perseus_lite_hud
{

    class HudElementBase
    {
    public:
        explicit HudElementBase(std::string name)
            : name_(std::move(name))
        {
        }
        virtual ~HudElementBase() = default;

        virtual void render(cv::Mat& frame) = 0;
        virtual bool is_ready() const = 0;

        const std::string& name() const { return name_; }

        bool enabled() const { return enabled_; }
        void set_enabled(bool e) { enabled_ = e; }

    protected:
        static void alpha_blend(const cv::Mat& overlay, cv::Mat& frame,
                                const cv::Point& origin, double alpha)
        {
            int x0 = origin.x;
            int y0 = origin.y;
            int w = overlay.cols;
            int h = overlay.rows;

            // Clamp to frame bounds
            int x1 = std::max(0, x0);
            int y1 = std::max(0, y0);
            int x2 = std::min(frame.cols, x0 + w);
            int y2 = std::min(frame.rows, y0 + h);
            if (x1 >= x2 || y1 >= y2)
                return;

            cv::Mat roi = frame(cv::Rect(x1, y1, x2 - x1, y2 - y1));
            cv::Mat src = overlay(cv::Rect(x1 - x0, y1 - y0, x2 - x1, y2 - y1));
            cv::addWeighted(src, alpha, roi, 1.0 - alpha, 0, roi);
        }

        std::string name_;
        bool enabled_ = true;
    };

}  // namespace perseus_lite_hud
