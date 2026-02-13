#include "perseus_lite_hud/hud_elements/servo_temperature.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace perseus_lite_hud {

ServoTemperature::ServoTemperature(const ServoTemperatureConfig &cfg)
    : HudElementBase("servo_temperature"), cfg_(cfg),
      start_time_(std::chrono::steady_clock::now()) {
  temps_.fill(0.0);
}

void ServoTemperature::render(cv::Mat &frame) {
  int pw = cfg_.width;
  int ph = cfg_.height;
  int margin = cfg_.margin;

  cv::Mat panel(ph, pw, CV_8UC3, cv::Scalar(15, 15, 15));

  // --- Corner brackets (MW2 targeting aesthetic) ---
  _draw_corner_brackets(panel);

  // --- Header ---
  int hdr_y = 16;
  cv::putText(panel, "SERVO TEMP", cv::Point(10, hdr_y),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1,
              cv::LINE_AA);

  if (has_data_) {
    double max_t = *std::max_element(temps_.begin(), temps_.end());
    char buf[32];
    std::snprintf(buf, sizeof(buf), "MAX:%.0f", max_t);
    // Right-align the max text
    int baseline = 0;
    auto sz = cv::getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, &baseline);
    cv::putText(panel, buf, cv::Point(pw - sz.width - 10, hdr_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.35,
                _color_for_temp(max_t), 1, cv::LINE_AA);
  }

  // Separator line
  cv::line(panel, cv::Point(8, hdr_y + 6), cv::Point(pw - 8, hdr_y + 6),
           cv::Scalar(0, 120, 0), 1);

  // --- Chassis wireframe with scanlines ---
  int chassis_w = 60;
  int chassis_h = 80;
  int chassis_x = (pw - chassis_w) / 2;
  int chassis_y = hdr_y + 14;
  _draw_chassis(panel, chassis_x, chassis_y, chassis_w, chassis_h);

  // --- Wheels ---
  int wheel_w = 28;
  int wheel_h = 24;
  int wheel_gap = 8; // gap between wheel and chassis

  // Vertical positions: front wheels top, rear wheels bottom
  int front_wy = chassis_y + 8;
  int rear_wy = chassis_y + chassis_h - wheel_h - 8;

  // Horizontal positions: left of chassis, right of chassis
  int left_wx = chassis_x - wheel_w - wheel_gap;
  int right_wx = chassis_x + chassis_w + wheel_gap;

  if (has_data_) {
    _draw_wheel(panel, left_wx, front_wy, wheel_w, wheel_h, temps_[0], "FL");
    _draw_wheel(panel, right_wx, front_wy, wheel_w, wheel_h, temps_[1], "FR");
    _draw_wheel(panel, left_wx, rear_wy, wheel_w, wheel_h, temps_[2], "RL");
    _draw_wheel(panel, right_wx, rear_wy, wheel_w, wheel_h, temps_[3], "RR");
  } else {
    // No data — show "--" in dim gray
    _draw_wheel(panel, left_wx, front_wy, wheel_w, wheel_h, -1.0, "FL");
    _draw_wheel(panel, right_wx, front_wy, wheel_w, wheel_h, -1.0, "FR");
    _draw_wheel(panel, left_wx, rear_wy, wheel_w, wheel_h, -1.0, "RL");
    _draw_wheel(panel, right_wx, rear_wy, wheel_w, wheel_h, -1.0, "RR");
  }

  // --- Temperature scale bar ---
  int scale_y = chassis_y + chassis_h + 20;
  int scale_h = 10;
  int scale_x = 14;
  int scale_w = pw - 28;
  _draw_scale_bar(panel, scale_x, scale_y, scale_w, scale_h);

  // Scale labels
  char lbl[16];
  cv::Scalar lbl_color(150, 150, 150);
  double font_scale = 0.28;

  std::snprintf(lbl, sizeof(lbl), "%.0f", cfg_.min_temp);
  cv::putText(panel, lbl, cv::Point(scale_x, scale_y + scale_h + 12),
              cv::FONT_HERSHEY_SIMPLEX, font_scale, lbl_color, 1, cv::LINE_AA);

  // Warning marker
  double warn_frac = (cfg_.warning_temp - cfg_.min_temp) / (cfg_.max_temp - cfg_.min_temp);
  int warn_x = scale_x + static_cast<int>(warn_frac * scale_w);
  std::snprintf(lbl, sizeof(lbl), "%.0f", cfg_.warning_temp);
  cv::putText(panel, lbl, cv::Point(warn_x - 6, scale_y + scale_h + 12),
              cv::FONT_HERSHEY_SIMPLEX, font_scale, lbl_color, 1, cv::LINE_AA);

  // Danger marker
  double dang_frac = (cfg_.danger_temp - cfg_.min_temp) / (cfg_.max_temp - cfg_.min_temp);
  int dang_x = scale_x + static_cast<int>(dang_frac * scale_w);
  std::snprintf(lbl, sizeof(lbl), "%.0f", cfg_.danger_temp);
  cv::putText(panel, lbl, cv::Point(dang_x - 6, scale_y + scale_h + 12),
              cv::FONT_HERSHEY_SIMPLEX, font_scale, lbl_color, 1, cv::LINE_AA);

  // Max + degree symbol
  std::snprintf(lbl, sizeof(lbl), "%.0f C", cfg_.max_temp);
  int baseline = 0;
  auto sz = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseline);
  cv::putText(panel, lbl,
              cv::Point(scale_x + scale_w - sz.width, scale_y + scale_h + 12),
              cv::FONT_HERSHEY_SIMPLEX, font_scale, lbl_color, 1, cv::LINE_AA);

  // --- Place panel bottom-right ---
  int x = frame.cols - pw - margin;
  int y = frame.rows - ph - margin;

  alpha_blend(panel, frame, cv::Point(x, y), cfg_.opacity);
}

bool ServoTemperature::is_ready() const {
  // Always ready — shows "--" when no data
  return true;
}

void ServoTemperature::set_temperatures(const std::array<double, 4> &temps) {
  temps_ = temps;
  has_data_ = true;
}

cv::Scalar ServoTemperature::_color_for_temp(double temp_c) const {
  // Cyan (≤30) → Green (31-40) → Yellow (41-50) → Orange (51-58) → Red (>65)
  // with linear interpolation between thresholds
  // OpenCV uses BGR

  if (temp_c <= 30.0) {
    return cv::Scalar(200, 200, 0); // Cyan
  }
  if (temp_c <= 40.0) {
    double t = (temp_c - 30.0) / 10.0;
    // Cyan(200,200,0) → Green(0,200,0)
    return cv::Scalar(200.0 * (1.0 - t), 200, 0);
  }
  if (temp_c <= 50.0) {
    double t = (temp_c - 40.0) / 10.0;
    // Green(0,200,0) → Yellow(0,200,200)
    return cv::Scalar(0, 200, 200.0 * t);
  }
  if (temp_c <= 58.0) {
    double t = (temp_c - 50.0) / 8.0;
    // Yellow(0,200,200) → Orange(0,140,255)
    return cv::Scalar(0, 200.0 - 60.0 * t, 200.0 + 55.0 * t);
  }
  if (temp_c <= 65.0) {
    double t = (temp_c - 58.0) / 7.0;
    // Orange(0,140,255) → Red(0,0,255)
    return cv::Scalar(0, 140.0 * (1.0 - t), 255);
  }
  // >65: full red
  return cv::Scalar(0, 0, 255);
}

bool ServoTemperature::_should_flash(double temp_c) const {
  if (temp_c < cfg_.danger_temp)
    return false;

  auto now = std::chrono::steady_clock::now();
  double elapsed =
      std::chrono::duration<double>(now - start_time_).count();
  double period = 1.0 / cfg_.flash_rate_hz;
  double phase = std::fmod(elapsed, period);
  return phase > (period / 2.0);
}

void ServoTemperature::_draw_corner_brackets(cv::Mat &panel) const {
  int w = panel.cols;
  int h = panel.rows;
  int blen = 18; // bracket arm length
  cv::Scalar bracket_color(0, 180, 0); // Green

  // Top-left
  cv::line(panel, cv::Point(2, 2), cv::Point(2 + blen, 2), bracket_color, 1);
  cv::line(panel, cv::Point(2, 2), cv::Point(2, 2 + blen), bracket_color, 1);

  // Top-right
  cv::line(panel, cv::Point(w - 3, 2), cv::Point(w - 3 - blen, 2), bracket_color, 1);
  cv::line(panel, cv::Point(w - 3, 2), cv::Point(w - 3, 2 + blen), bracket_color, 1);

  // Bottom-left
  cv::line(panel, cv::Point(2, h - 3), cv::Point(2 + blen, h - 3), bracket_color, 1);
  cv::line(panel, cv::Point(2, h - 3), cv::Point(2, h - 3 - blen), bracket_color, 1);

  // Bottom-right
  cv::line(panel, cv::Point(w - 3, h - 3), cv::Point(w - 3 - blen, h - 3), bracket_color, 1);
  cv::line(panel, cv::Point(w - 3, h - 3), cv::Point(w - 3, h - 3 - blen), bracket_color, 1);
}

void ServoTemperature::_draw_chassis(cv::Mat &panel, int cx, int cy, int cw,
                                     int ch) const {
  // Chassis outline in green
  cv::Scalar chassis_color(0, 160, 0);
  cv::rectangle(panel, cv::Point(cx, cy), cv::Point(cx + cw, cy + ch),
                chassis_color, 1);

  // Horizontal scanlines (CRT feel) — every 6 pixels
  for (int sy = cy + 6; sy < cy + ch; sy += 6) {
    cv::line(panel, cv::Point(cx + 1, sy), cv::Point(cx + cw - 1, sy),
             cv::Scalar(0, 80, 0), 1);
  }

  // Forward arrow (chevron) at top-center of chassis
  int arrow_cx = cx + cw / 2;
  int arrow_y = cy + ch / 2 - 6;
  cv::line(panel, cv::Point(arrow_cx - 8, arrow_y + 8),
           cv::Point(arrow_cx, arrow_y), cv::Scalar(0, 220, 0), 1, cv::LINE_AA);
  cv::line(panel, cv::Point(arrow_cx, arrow_y),
           cv::Point(arrow_cx + 8, arrow_y + 8), cv::Scalar(0, 220, 0), 1,
           cv::LINE_AA);
}

void ServoTemperature::_draw_wheel(cv::Mat &panel, int x, int y, int w, int h,
                                   double temp,
                                   const std::string &label) const {
  bool no_data = (temp < 0.0);

  cv::Scalar fill_color;
  if (no_data) {
    fill_color = cv::Scalar(60, 60, 60); // Dim gray
  } else if (_should_flash(temp)) {
    // Flash between red and white
    auto now = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration<double>(now - start_time_).count();
    double period = 1.0 / cfg_.flash_rate_hz;
    double phase = std::fmod(elapsed, period);
    // First quarter: white, rest: red
    if (phase < period / 4.0) {
      fill_color = cv::Scalar(200, 200, 255); // White-ish
    } else {
      fill_color = cv::Scalar(0, 0, 255); // Red
    }
  } else {
    fill_color = _color_for_temp(temp);
  }

  // Filled wheel block
  cv::rectangle(panel, cv::Point(x, y), cv::Point(x + w, y + h), fill_color,
                cv::FILLED);

  // Wheel border
  cv::rectangle(panel, cv::Point(x, y), cv::Point(x + w, y + h),
                cv::Scalar(100, 100, 100), 1);

  // Temp text centered on wheel
  char buf[16];
  if (no_data) {
    std::snprintf(buf, sizeof(buf), "--");
  } else {
    std::snprintf(buf, sizeof(buf), "%.0f", temp);
  }

  int baseline = 0;
  auto text_sz =
      cv::getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, &baseline);
  int tx = x + (w - text_sz.width) / 2;
  int ty = y + (h + text_sz.height) / 2;

  // Dark text on bright fills, bright on dark
  cv::Scalar text_color;
  if (no_data) {
    text_color = cv::Scalar(120, 120, 120);
  } else {
    // Use dark text for readability on bright fills
    text_color = cv::Scalar(0, 0, 0);
  }
  cv::putText(panel, buf, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, 0.35,
              text_color, 1, cv::LINE_AA);

  // Label below (or above) the wheel
  auto lbl_sz =
      cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.3, 1, &baseline);
  int lx = x + (w - lbl_sz.width) / 2;
  int ly = y + h + lbl_sz.height + 3;

  cv::Scalar lbl_color = no_data ? cv::Scalar(80, 80, 80)
                                 : cv::Scalar(180, 180, 180);
  cv::putText(panel, label, cv::Point(lx, ly), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              lbl_color, 1, cv::LINE_AA);
}

void ServoTemperature::_draw_scale_bar(cv::Mat &panel, int x, int y, int w,
                                       int h) const {
  // Draw horizontal gradient bar from min_temp to max_temp
  for (int px = 0; px < w; ++px) {
    double frac = static_cast<double>(px) / (w - 1);
    double temp = cfg_.min_temp + frac * (cfg_.max_temp - cfg_.min_temp);
    cv::Scalar color = _color_for_temp(temp);
    cv::line(panel, cv::Point(x + px, y), cv::Point(x + px, y + h), color, 1);
  }

  // Border
  cv::rectangle(panel, cv::Point(x, y), cv::Point(x + w, y + h),
                cv::Scalar(80, 80, 80), 1);
}

} // namespace perseus_lite_hud
