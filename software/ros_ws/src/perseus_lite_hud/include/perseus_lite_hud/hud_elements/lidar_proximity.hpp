#pragma once

#include "perseus_lite_hud/hud_element_base.hpp"
#include <array>
#include <chrono>

namespace perseus_lite_hud {

struct LidarProximityConfig {
  double danger_dist = 0.5;
  double warning_dist = 1.5;
  double clear_dist = 3.0;
  int bar_thickness = 8;
  double opacity = 0.6;
  double flash_rate_hz = 4.0;
};

class LidarProximity : public HudElementBase {
public:
  explicit LidarProximity(const LidarProximityConfig &cfg = {});

  void render(cv::Mat &frame) override;
  bool is_ready() const override;

  // sector_mins: 8 sectors (0=front, clockwise), each value is min range in meters
  void set_sector_ranges(const std::array<double, 8> &sector_mins);

  static std::array<double, 8>
  compute_sectors(const float *ranges, size_t count, double angle_min,
                  double angle_increment);

private:
  cv::Scalar _color_for_range(double range) const;
  bool _should_flash(double range) const;

  LidarProximityConfig cfg_;
  std::array<double, 8> sector_mins_{};
  bool has_data_ = false;
  std::chrono::steady_clock::time_point start_time_;
};

} // namespace perseus_lite_hud
