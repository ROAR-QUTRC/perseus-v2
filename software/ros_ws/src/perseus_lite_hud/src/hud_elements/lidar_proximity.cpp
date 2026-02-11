#include "perseus_lite_hud/hud_elements/lidar_proximity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace perseus_lite_hud {

LidarProximity::LidarProximity(const LidarProximityConfig &cfg)
    : HudElementBase("lidar_proximity"), cfg_(cfg),
      start_time_(std::chrono::steady_clock::now()) {
  sector_mins_.fill(std::numeric_limits<double>::infinity());
}

void LidarProximity::render(cv::Mat &frame) {
  if (!has_data_)
    return;

  int w = frame.cols;
  int h = frame.rows;
  int t = cfg_.bar_thickness;

  // Sector layout (8 sectors, 45° each, starting from front=0, clockwise):
  //   0=front, 1=front-right, 2=right, 3=back-right
  //   4=back,  5=back-left,   6=left,  7=front-left
  //
  // Edge mapping:
  //   Top edge:    sectors 7 (left half), 0 (center), 1 (right half)
  //   Right edge:  sectors 1 (top), 2 (center), 3 (bottom)
  //   Bottom edge: sectors 3 (left), 4 (center), 5 (right)
  //   Left edge:   sectors 5 (bottom), 6 (center), 7 (top)

  struct BarDef {
    int sector;
    cv::Point p1, p2;
  };

  // Top edge (left to right): sector 7, 0, 1
  int third_w = w / 3;
  int third_h = h / 3;

  BarDef bars[] = {
      // Top edge
      {7, {0, 0}, {third_w, t}},
      {0, {third_w, 0}, {2 * third_w, t}},
      {1, {2 * third_w, 0}, {w, t}},
      // Right edge
      {1, {w - t, 0}, {w, third_h}},
      {2, {w - t, third_h}, {w, 2 * third_h}},
      {3, {w - t, 2 * third_h}, {w, h}},
      // Bottom edge
      {5, {0, h - t}, {third_w, h}},
      {4, {third_w, h - t}, {2 * third_w, h}},
      {3, {2 * third_w, h - t}, {w, h}},
      // Left edge
      {7, {0, 0}, {t, third_h}},
      {6, {0, third_h}, {t, 2 * third_h}},
      {5, {0, 2 * third_h}, {t, h}},
  };

  for (const auto &bar : bars) {
    double range = sector_mins_[bar.sector];
    cv::Scalar color = _color_for_range(range);

    if (_should_flash(range)) {
      // Flash: skip drawing on alternate half-periods
      continue;
    }

    cv::Mat roi = frame(cv::Rect(bar.p1, bar.p2));
    cv::Mat overlay(roi.size(), CV_8UC3, color);
    cv::addWeighted(overlay, cfg_.opacity, roi, 1.0 - cfg_.opacity, 0, roi);
  }
}

bool LidarProximity::is_ready() const { return has_data_; }

void LidarProximity::set_sector_ranges(
    const std::array<double, 8> &sector_mins) {
  sector_mins_ = sector_mins;
  has_data_ = true;
}

std::array<double, 8>
LidarProximity::compute_sectors(const float *ranges, size_t count,
                                double angle_min, double angle_increment) {
  std::array<double, 8> mins;
  mins.fill(std::numeric_limits<double>::infinity());

  for (size_t i = 0; i < count; ++i) {
    float r = ranges[i];
    if (!std::isfinite(r) || r <= 0.0f)
      continue;

    double angle = angle_min + i * angle_increment;

    // Normalize to [0, 2π). Robot frame: 0 = forward, positive = left (CCW)
    // We want sectors: 0=front, 1=front-right, ... clockwise
    // So negate angle to go clockwise, then normalize
    double sector_angle = std::fmod(-angle + 2.0 * M_PI, 2.0 * M_PI);
    int sector = static_cast<int>(sector_angle / (M_PI / 4.0)) % 8;

    if (r < mins[sector]) {
      mins[sector] = r;
    }
  }

  return mins;
}

cv::Scalar LidarProximity::_color_for_range(double range) const {
  if (range < cfg_.danger_dist)
    return cv::Scalar(0, 0, 255); // Red
  if (range < cfg_.warning_dist)
    return cv::Scalar(0, 200, 255); // Yellow
  if (range < cfg_.clear_dist)
    return cv::Scalar(0, 255, 0); // Green
  return cv::Scalar(0, 180, 0);   // Dim green
}

bool LidarProximity::_should_flash(double range) const {
  if (range >= cfg_.danger_dist)
    return false;

  auto now = std::chrono::steady_clock::now();
  double elapsed =
      std::chrono::duration<double>(now - start_time_).count();
  double period = 1.0 / cfg_.flash_rate_hz;
  double phase = std::fmod(elapsed, period);
  return phase > (period / 2.0);
}

} // namespace perseus_lite_hud
