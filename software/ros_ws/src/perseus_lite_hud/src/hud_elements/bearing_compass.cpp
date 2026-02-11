#include "perseus_lite_hud/hud_elements/bearing_compass.hpp"

#include <cmath>
#include <string>

namespace perseus_lite_hud
{

    BearingCompass::BearingCompass(const BearingCompassConfig& cfg)
        : HudElementBase("bearing_compass"),
          cfg_(cfg)
    {
    }

    void BearingCompass::render(cv::Mat& frame)
    {
        if (!has_data_)
            return;

        int strip_w = cfg_.width;
        int strip_h = cfg_.height;

        // Create semi-transparent overlay strip
        cv::Mat strip(strip_h, strip_w, CV_8UC3, cv::Scalar(20, 20, 20));

        _draw_graduation(strip, heading_deg_);

        // Position: top center
        int x = (frame.cols - strip_w) / 2;
        int y = 8;

        alpha_blend(strip, frame, cv::Point(x, y), cfg_.opacity);

        // Center indicator triangle
        _draw_indicator(frame, cv::Point(x + strip_w / 2, y));
    }

    bool BearingCompass::is_ready() const { return has_data_; }

    void BearingCompass::set_heading(double heading_deg)
    {
        heading_deg_ = heading_deg;
        has_data_ = true;
    }

    void BearingCompass::_draw_graduation(cv::Mat& strip, double center_deg)
    {
        int w = strip.cols;
        int h = strip.rows;
        double deg_per_px = static_cast<double>(cfg_.degrees_visible) / w;

        double left_deg = center_deg - cfg_.degrees_visible / 2.0;

        // Cardinal/intercardinal labels
        static const char* cardinal_labels[] = {"N", "NE", "E", "SE",
                                                "S", "SW", "W", "NW"};
        static const double cardinal_angles[] = {0, 45, 90, 135, 180, 225, 270, 315};

        for (int px = 0; px < w; ++px)
        {
            double deg = left_deg + px * deg_per_px;
            // Normalize to [0, 360)
            double norm = std::fmod(deg + 3600.0, 360.0);
            int ideg = static_cast<int>(std::round(norm));

            if (ideg % 5 == 0)
            {
                // Tick mark
                int tick_h;
                cv::Scalar color;

                if (ideg % 15 == 0)
                {
                    // Major tick
                    tick_h = h / 2;
                    color = cv::Scalar(200, 200, 200);

                    // Degree label
                    std::string label = std::to_string(ideg);

                    // Check for cardinal labels
                    for (int c = 0; c < 8; ++c)
                    {
                        if (ideg == static_cast<int>(cardinal_angles[c]))
                        {
                            label = cardinal_labels[c];
                            color = cv::Scalar(0, 200, 255);  // Orange for cardinals
                            break;
                        }
                    }

                    int baseline = 0;
                    cv::Size text_sz = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                                       0.35, 1, &baseline);
                    int tx = px - text_sz.width / 2;
                    int ty = h - 4;
                    if (tx >= 0 && tx + text_sz.width < w)
                    {
                        cv::putText(strip, label, cv::Point(tx, ty),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.35, color, 1,
                                    cv::LINE_AA);
                    }
                }
                else
                {
                    // Minor tick
                    tick_h = h / 4;
                    color = cv::Scalar(120, 120, 120);
                }

                cv::line(strip, cv::Point(px, 0), cv::Point(px, tick_h), color, 1);
            }
        }

        // Center heading text (large)
        int display_heading = static_cast<int>(std::round(
            std::fmod(center_deg + 360.0, 360.0)));
        std::string hdg_str = std::to_string(display_heading) + "\xC2\xB0";
        int baseline = 0;
        cv::Size hdg_sz = cv::getTextSize(hdg_str, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                                          1, &baseline);
        cv::putText(strip, hdg_str,
                    cv::Point(w / 2 - hdg_sz.width / 2, h / 2 + hdg_sz.height / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1,
                    cv::LINE_AA);
    }

    void BearingCompass::_draw_indicator(cv::Mat& frame,
                                         const cv::Point& center_top)
    {
        // Small downward-pointing triangle above the strip
        int size = 6;
        std::vector<cv::Point> tri = {
            cv::Point(center_top.x, center_top.y + size),
            cv::Point(center_top.x - size, center_top.y - 2),
            cv::Point(center_top.x + size, center_top.y - 2),
        };
        cv::fillConvexPoly(frame, tri, cv::Scalar(0, 255, 0), cv::LINE_AA);
    }

}  // namespace perseus_lite_hud
