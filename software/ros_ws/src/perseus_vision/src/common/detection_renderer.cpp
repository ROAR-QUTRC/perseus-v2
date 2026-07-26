/// @file detection_renderer.cpp
/// @brief Implementation of the shared detection overlay rendering.

#include "perseus_vision/common/detection_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>

namespace perseus_vision
{
    namespace
    {
        /// @brief Line thickness of a detection bounding polygon, in pixels.
        constexpr int POLYGON_THICKNESS = 2;
        /// @brief Height of the label text above the polygon's topmost corner, in pixels.
        constexpr int LABEL_Y_OFFSET_PX = 8;
        /// @brief Font scale of the detection label.
        constexpr double LABEL_FONT_SCALE = 0.5;
        /// @brief Stroke thickness of the detection label.
        constexpr int LABEL_THICKNESS = 1;
        /// @brief Decimal places shown for a detection's confidence.
        constexpr int CONFIDENCE_PRECISION = 2;
        /// @brief Largest value a single 8-bit colour channel can hold.
        constexpr double PIXEL_VALUE_MAX = 255.0;
        /// @brief Colour used when a detection carries no usable colour of its own.
        const cv::Scalar FALLBACK_COLOR(0, 255, 0);

        /// @brief Converts a detection's RGBA colour into an OpenCV BGR scalar.
        /// @param color Detection colour, components in [0, 1].
        /// @return The equivalent BGR scalar, or a fallback if the colour is unset.
        cv::Scalar bgr_from_color(const std_msgs::msg::ColorRGBA& color)
        {
            // A fully transparent colour is treated as "not set" so that a detection
            // built without an explicit colour still renders visibly.
            if (color.a <= 0.0f)
            {
                return FALLBACK_COLOR;
            }

            return cv::Scalar(
                static_cast<double>(color.b) * PIXEL_VALUE_MAX,
                static_cast<double>(color.g) * PIXEL_VALUE_MAX,
                static_cast<double>(color.r) * PIXEL_VALUE_MAX);
        }

        /// @brief Converts a detection polygon into OpenCV image points.
        /// @param polygon Detection bounding polygon.
        /// @return The polygon's corners, rounded to whole pixels.
        std::vector<cv::Point> points_from_polygon(const geometry_msgs::msg::Polygon& polygon)
        {
            std::vector<cv::Point> points;
            points.reserve(polygon.points.size());
            for (const auto& point : polygon.points)
            {
                points.emplace_back(static_cast<int>(std::lround(point.x)),
                                    static_cast<int>(std::lround(point.y)));
            }
            return points;
        }

        /// @brief Formats the text drawn beside a detection.
        /// @param detection Detection to describe.
        /// @return The class label followed by its confidence, such as "cube_blue 0.97".
        std::string format_label(const perseus_interfaces::msg::Detection& detection)
        {
            std::ostringstream label_stream;
            label_stream << detection.class_label << " "
                         << std::fixed << std::setprecision(CONFIDENCE_PRECISION)
                         << detection.confidence;
            return label_stream.str();
        }
    }  // namespace

    void draw_detection(cv::Mat& frame, const perseus_interfaces::msg::Detection& detection)
    {
        const std::vector<cv::Point> points = points_from_polygon(detection.bounding_box);
        if (points.empty())
        {
            return;
        }

        const cv::Scalar color = bgr_from_color(detection.color);

        // The polygon is implicitly closed, so a 4 point box draws 4 connected edges.
        cv::polylines(frame, points, true, color, POLYGON_THICKNESS);

        // Anchor the label to the topmost corner so it tracks rotated boxes.
        const auto topmost = std::min_element(
            points.begin(), points.end(),
            [](const cv::Point& left, const cv::Point& right)
            { return left.y < right.y; });

        cv::putText(frame, format_label(detection),
                    cv::Point(topmost->x, topmost->y - LABEL_Y_OFFSET_PX),
                    cv::FONT_HERSHEY_SIMPLEX, LABEL_FONT_SCALE, color, LABEL_THICKNESS);
    }

    void draw_detections(cv::Mat& frame, const perseus_interfaces::msg::DetectionArray& detections)
    {
        for (const auto& detection : detections.detections)
        {
            draw_detection(frame, detection);
        }
    }

    geometry_msgs::msg::Polygon polygon_from_rect(const cv::Rect& bounding_box)
    {
        const float left = static_cast<float>(bounding_box.x);
        const float top = static_cast<float>(bounding_box.y);
        const float right = static_cast<float>(bounding_box.x + bounding_box.width);
        const float bottom = static_cast<float>(bounding_box.y + bounding_box.height);

        geometry_msgs::msg::Polygon polygon;
        polygon.points.resize(4);
        polygon.points[0].x = left;
        polygon.points[0].y = top;
        polygon.points[1].x = right;
        polygon.points[1].y = top;
        polygon.points[2].x = right;
        polygon.points[2].y = bottom;
        polygon.points[3].x = left;
        polygon.points[3].y = bottom;
        return polygon;
    }

    geometry_msgs::msg::Polygon polygon_from_corners(const std::vector<cv::Point2f>& corner_points)
    {
        geometry_msgs::msg::Polygon polygon;
        polygon.points.reserve(corner_points.size());
        for (const auto& corner_point : corner_points)
        {
            geometry_msgs::msg::Point32 point;
            point.x = corner_point.x;
            point.y = corner_point.y;
            polygon.points.push_back(point);
        }
        return polygon;
    }

    std_msgs::msg::ColorRGBA color_from_bgr(const cv::Scalar& bgr_color, float alpha)
    {
        std_msgs::msg::ColorRGBA color;
        color.b = static_cast<float>(bgr_color[0] / PIXEL_VALUE_MAX);
        color.g = static_cast<float>(bgr_color[1] / PIXEL_VALUE_MAX);
        color.r = static_cast<float>(bgr_color[2] / PIXEL_VALUE_MAX);
        color.a = alpha;
        return color;
    }

}  // namespace perseus_vision
