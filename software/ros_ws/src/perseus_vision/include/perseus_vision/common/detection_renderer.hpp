#pragma once

/// @file detection_renderer.hpp
/// @brief Shared rendering of detection overlays onto camera images.
///
/// Used both by the overlay node, which annotates the live image stream, and by
/// the detector capture services, which annotate a still frame on request. Keeping
/// one implementation means a captured image looks exactly like the live overlay.

#include <geometry_msgs/msg/polygon.hpp>
#include <opencv2/core.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <vector>

#include "perseus_interfaces/msg/detection.hpp"
#include "perseus_interfaces/msg/detection_array.hpp"

namespace perseus_vision
{
    /// @brief Draws one detection's bounding polygon and label onto an image.
    /// @param frame Image to draw onto. Modified in place.
    /// @param detection Detection to render, whose colour is taken from the message.
    void draw_detection(cv::Mat& frame, const perseus_interfaces::msg::Detection& detection);

    /// @brief Draws every detection in an array onto an image.
    /// @param frame Image to draw onto. Modified in place.
    /// @param detections Detections to render, in array order.
    void draw_detections(cv::Mat& frame, const perseus_interfaces::msg::DetectionArray& detections);

    /// @brief Builds an axis-aligned detection polygon from a bounding rectangle.
    /// @param bounding_box Bounding box in image coordinates.
    /// @return The four corners, clockwise from the top left.
    geometry_msgs::msg::Polygon polygon_from_rect(const cv::Rect& bounding_box);

    /// @brief Builds a detection polygon from an arbitrary set of image points.
    /// @param corner_points Corners in image coordinates, in draw order.
    /// @return The equivalent polygon.
    geometry_msgs::msg::Polygon polygon_from_corners(const std::vector<cv::Point2f>& corner_points);

    /// @brief Builds a detection colour from 8-bit BGR components.
    /// @param bgr_color Colour in OpenCV BGR order, components in [0, 255].
    /// @param alpha Opacity in [0, 1].
    /// @return The equivalent RGBA colour message.
    std_msgs::msg::ColorRGBA color_from_bgr(const cv::Scalar& bgr_color, float alpha = 1.0f);

}  // namespace perseus_vision
