#pragma once

#include "perseus_lite_hud/hud_element_base.hpp"

namespace perseus_lite_hud {

struct OdometerConfig {
  double opacity = 0.8;
  int digit_width = 28;
  int digit_height = 40;
};

class Odometer : public HudElementBase {
public:
  explicit Odometer(const OdometerConfig &cfg = {});

  void render(cv::Mat &frame) override;
  bool is_ready() const override;

  void update_position(double x, double y);
  void reset();

  double distance() const { return total_distance_m_; }

private:
  void _draw_digit_box(cv::Mat &frame, const cv::Point &origin, char c);

  OdometerConfig cfg_;
  double total_distance_m_ = 0.0;
  double prev_x_ = 0.0;
  double prev_y_ = 0.0;
  bool has_prev_ = false;
  bool has_data_ = false;
};

} // namespace perseus_lite_hud
