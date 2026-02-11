#include "perseus_lite_hud/hud_elements/odometer.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace perseus_lite_hud {

Odometer::Odometer(const OdometerConfig &cfg)
    : HudElementBase("odometer"), cfg_(cfg) {}

void Odometer::render(cv::Mat &frame) {
  if (!has_data_)
    return;

  // Format: "0087.3 m" — 4 integer digits, 1 decimal
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%06.1f", total_distance_m_);
  std::string dist_str(buf);

  int num_chars = static_cast<int>(dist_str.size());
  int total_w = num_chars * cfg_.digit_width + cfg_.digit_width * 2; // + " m"
  int total_h = cfg_.digit_height;

  cv::Mat panel(total_h + 8, total_w + 8, CV_8UC3, cv::Scalar(20, 20, 20));

  // Draw each digit in its own box
  int x_offset = 4;
  for (char c : dist_str) {
    _draw_digit_box(panel, cv::Point(x_offset, 4), c);
    x_offset += cfg_.digit_width;
  }

  // " m" suffix
  cv::putText(panel, " m", cv::Point(x_offset + 2, 4 + cfg_.digit_height - 8),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1,
              cv::LINE_AA);

  // Position: bottom center
  int px = (frame.cols - panel.cols) / 2;
  int py = frame.rows - panel.rows - 10;

  alpha_blend(panel, frame, cv::Point(px, py), cfg_.opacity);
}

bool Odometer::is_ready() const { return has_data_; }

void Odometer::update_position(double x, double y) {
  if (has_prev_) {
    double dx = x - prev_x_;
    double dy = y - prev_y_;
    double dist = std::sqrt(dx * dx + dy * dy);

    // Filter out teleports/jumps (>2m in one step is suspicious)
    if (dist < 2.0) {
      total_distance_m_ += dist;
    }
  }

  prev_x_ = x;
  prev_y_ = y;
  has_prev_ = true;
  has_data_ = true;
}

void Odometer::reset() {
  total_distance_m_ = 0.0;
  has_prev_ = false;
}

void Odometer::_draw_digit_box(cv::Mat &frame, const cv::Point &origin,
                               char c) {
  int w = cfg_.digit_width;
  int h = cfg_.digit_height;

  // Dark background box
  cv::rectangle(frame, origin, cv::Point(origin.x + w - 2, origin.y + h),
                cv::Scalar(10, 10, 10), cv::FILLED);

  // Border
  cv::rectangle(frame, origin, cv::Point(origin.x + w - 2, origin.y + h),
                cv::Scalar(60, 60, 60), 1);

  // Character
  cv::Scalar color = (c == '.') ? cv::Scalar(100, 100, 100)
                                : cv::Scalar(255, 255, 255);

  std::string s(1, c);
  int baseline = 0;
  cv::Size sz =
      cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
  int tx = origin.x + (w - 2 - sz.width) / 2;
  int ty = origin.y + (h + sz.height) / 2;

  cv::putText(frame, s, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.55,
              color, 1, cv::LINE_AA);
}

} // namespace perseus_lite_hud
