#include "perseus_lite_hud/hud_elements/velocity_gauge.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace perseus_lite_hud {

VelocityGauge::VelocityGauge(const VelocityGaugeConfig &cfg)
    : HudElementBase("velocity_gauge"), cfg_(cfg) {}

void VelocityGauge::render(cv::Mat &frame) {
  if (!has_data_)
    return;

  int gauge_w = cfg_.width;
  int gauge_h = cfg_.height;
  int margin = cfg_.margin;

  // Create gauge background
  cv::Mat gauge(gauge_h, gauge_w, CV_8UC3, cv::Scalar(20, 20, 20));

  // === Linear velocity bar (left half, vertical) ===
  int bar_w = gauge_w / 3;
  int bar_h = gauge_h - 20; // Leave room for labels
  int bar_x = 4;
  int bar_y = 10;
  int center_y = bar_y + bar_h / 2;

  // Bar outline
  cv::rectangle(gauge, cv::Point(bar_x, bar_y),
                cv::Point(bar_x + bar_w, bar_y + bar_h),
                cv::Scalar(100, 100, 100), 1);

  // Center line (zero)
  cv::line(gauge, cv::Point(bar_x, center_y),
           cv::Point(bar_x + bar_w, center_y), cv::Scalar(150, 150, 150), 1);

  // Fill bar
  double lin_ratio =
      std::clamp(linear_ / cfg_.max_linear, -1.0, 1.0);
  int fill_h = static_cast<int>(std::abs(lin_ratio) * (bar_h / 2));

  if (lin_ratio > 0) {
    // Forward = green, fill upward from center
    cv::rectangle(gauge, cv::Point(bar_x + 1, center_y - fill_h),
                  cv::Point(bar_x + bar_w - 1, center_y),
                  cv::Scalar(0, 200, 0), cv::FILLED);
  } else if (lin_ratio < 0) {
    // Reverse = red, fill downward from center
    cv::rectangle(gauge, cv::Point(bar_x + 1, center_y),
                  cv::Point(bar_x + bar_w - 1, center_y + fill_h),
                  cv::Scalar(0, 0, 200), cv::FILLED);
  }

  // === Angular velocity indicator (right side, horizontal) ===
  int ang_w = gauge_w - bar_w - 16;
  int ang_h = 16;
  int ang_x = bar_x + bar_w + 8;
  int ang_y = center_y - ang_h / 2;
  int ang_cx = ang_x + ang_w / 2;

  cv::rectangle(gauge, cv::Point(ang_x, ang_y),
                cv::Point(ang_x + ang_w, ang_y + ang_h),
                cv::Scalar(100, 100, 100), 1);

  // Center line
  cv::line(gauge, cv::Point(ang_cx, ang_y),
           cv::Point(ang_cx, ang_y + ang_h), cv::Scalar(150, 150, 150), 1);

  double ang_ratio =
      std::clamp(angular_ / cfg_.max_angular, -1.0, 1.0);
  int fill_w = static_cast<int>(std::abs(ang_ratio) * (ang_w / 2));

  cv::Scalar ang_color(200, 200, 0); // Cyan-ish
  if (ang_ratio > 0) {
    // Left turn (positive angular = CCW) — fill left from center
    cv::rectangle(gauge, cv::Point(ang_cx - fill_w, ang_y + 1),
                  cv::Point(ang_cx, ang_y + ang_h - 1), ang_color,
                  cv::FILLED);
  } else if (ang_ratio < 0) {
    // Right turn — fill right from center
    cv::rectangle(gauge, cv::Point(ang_cx, ang_y + 1),
                  cv::Point(ang_cx + fill_w, ang_y + ang_h - 1), ang_color,
                  cv::FILLED);
  }

  // Speed text
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.2f", linear_);
  cv::putText(gauge, buf, cv::Point(ang_x, ang_y - 4),
              cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 200, 0), 1,
              cv::LINE_AA);

  cv::putText(gauge, "m/s", cv::Point(ang_x, ang_y - 14),
              cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(150, 150, 150), 1,
              cv::LINE_AA);

  // Position: bottom left
  int x = margin;
  int y = frame.rows - gauge_h - margin;

  alpha_blend(gauge, frame, cv::Point(x, y), cfg_.opacity);
}

bool VelocityGauge::is_ready() const { return has_data_; }

void VelocityGauge::set_velocity(double linear, double angular) {
  linear_ = linear;
  angular_ = angular;
  has_data_ = true;
}

} // namespace perseus_lite_hud
