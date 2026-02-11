#include "perseus_lite_hud/hud_elements/pip_map.hpp"

#include <cmath>

namespace perseus_lite_hud {

PipMap::PipMap(const PipMapConfig &cfg)
    : HudElementBase("pip_map"), cfg_(cfg) {}

void PipMap::render(cv::Mat &frame) {
  if (!has_map_ || !has_pose_)
    return;

  if (map_image_dirty_) {
    _rebuild_map_image();
  }

  if (map_image_.empty())
    return;

  int pip_size = cfg_.size;
  double resolution = map_->info.resolution;
  double origin_x = map_->info.origin.position.x;
  double origin_y = map_->info.origin.position.y;

  // Robot position in map image coordinates
  double robot_px = (robot_x_ - origin_x) / resolution;
  double robot_py =
      map_image_.rows - 1 - (robot_y_ - origin_y) / resolution;

  // Radius in pixels
  double radius_px = cfg_.radius_m / resolution;

  // Extract region around robot
  int crop_size = static_cast<int>(2.0 * radius_px);
  int x1 = static_cast<int>(robot_px - radius_px);
  int y1 = static_cast<int>(robot_py - radius_px);

  // Create output (black if out of bounds)
  cv::Mat crop(crop_size, crop_size, CV_8UC3, cv::Scalar(40, 40, 40));

  // Copy valid region
  int src_x1 = std::max(0, x1);
  int src_y1 = std::max(0, y1);
  int src_x2 = std::min(map_image_.cols, x1 + crop_size);
  int src_y2 = std::min(map_image_.rows, y1 + crop_size);

  if (src_x1 < src_x2 && src_y1 < src_y2) {
    cv::Mat src_roi = map_image_(
        cv::Rect(src_x1, src_y1, src_x2 - src_x1, src_y2 - src_y1));
    cv::Mat dst_roi = crop(cv::Rect(src_x1 - x1, src_y1 - y1,
                                    src_x2 - src_x1, src_y2 - src_y1));
    src_roi.copyTo(dst_roi);
  }

  // Resize to PIP size
  cv::Mat pip;
  cv::resize(crop, pip, cv::Size(pip_size, pip_size), 0, 0,
             cv::INTER_NEAREST);

  // Draw robot arrow (rotates with heading — north-up map)
  int cx = pip_size / 2;
  int cy = pip_size / 2;
  int arrow_len = 12;

  // Robot yaw: 0=east in ROS, we want arrow pointing in heading direction
  // Map is north-up, so north = up = -Y in image coords
  double angle = -robot_yaw_ + M_PI / 2.0; // Convert to image coords
  cv::Point tip(cx + static_cast<int>(arrow_len * std::cos(angle)),
                cy - static_cast<int>(arrow_len * std::sin(angle)));
  cv::arrowedLine(pip, cv::Point(cx, cy), tip, cv::Scalar(0, 255, 0), 2,
                  cv::LINE_AA, 0, 0.4);

  // Border
  cv::rectangle(pip, cv::Point(0, 0), cv::Point(pip_size - 1, pip_size - 1),
                cv::Scalar(180, 180, 180), 1);

  // "MAP" label
  cv::putText(pip, "MAP", cv::Point(4, pip_size - 6),
              cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(180, 180, 180), 1,
              cv::LINE_AA);

  // Position: top right
  int x = frame.cols - pip_size - cfg_.margin;
  int y = cfg_.margin;

  alpha_blend(pip, frame, cv::Point(x, y), cfg_.opacity);
}

bool PipMap::is_ready() const { return has_map_ && has_pose_; }

void PipMap::set_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
  map_ = std::move(map);
  has_map_ = true;
  map_image_dirty_ = true;
}

void PipMap::set_robot_pose(double x, double y, double yaw) {
  robot_x_ = x;
  robot_y_ = y;
  robot_yaw_ = yaw;
  has_pose_ = true;
}

void PipMap::_rebuild_map_image() {
  if (!map_)
    return;

  int w = static_cast<int>(map_->info.width);
  int h = static_cast<int>(map_->info.height);

  if (w <= 0 || h <= 0)
    return;

  map_image_ = cv::Mat(h, w, CV_8UC3);

  for (int row = 0; row < h; ++row) {
    for (int col = 0; col < w; ++col) {
      // OccupancyGrid: row-major, origin at bottom-left
      // Image: origin at top-left, so flip Y
      int grid_idx = (h - 1 - row) * w + col;
      int8_t val = map_->data[grid_idx];

      cv::Vec3b color;
      if (val < 0) {
        // Unknown
        color = cv::Vec3b(80, 80, 80);
      } else if (val == 0) {
        // Free
        color = cv::Vec3b(200, 200, 200);
      } else {
        // Occupied — darker for higher occupancy
        uint8_t intensity =
            static_cast<uint8_t>(255 - std::min(static_cast<int>(val), 100) * 2);
        color = cv::Vec3b(intensity / 3, intensity / 3, intensity);
      }
      map_image_.at<cv::Vec3b>(row, col) = color;
    }
  }

  map_image_dirty_ = false;
}

} // namespace perseus_lite_hud
