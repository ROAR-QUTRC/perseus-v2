/// @file detection_overlay.cpp
/// @brief Implementation of the real-time detection overlay node.

#include "perseus_vision/detection_overlay/detection_overlay.hpp"

#include <functional>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>
#include <utility>

#include "perseus_vision/common/detection_renderer.hpp"

namespace perseus_vision
{
    namespace
    {
        /// @brief Queue depth used for the image topics.
        ///
        /// Deliberately shallow: if the node cannot keep up, dropping older frames keeps
        /// the overlay current rather than publishing a growing backlog of stale images.
        constexpr int IMAGE_QOS_DEPTH = 1;
        /// @brief Queue depth used for the detection topics.
        constexpr int DETECTION_QOS_DEPTH = 5;
        /// @brief JPEG quality used when re-encoding the annotated compressed image.
        constexpr int JPEG_QUALITY = 90;
        /// @brief Minimum gap between repeated warning logs, in milliseconds.
        constexpr int64_t LOG_THROTTLE_MS = 2000;
        /// @brief Left margin used for the staleness readout, in pixels.
        constexpr int STALENESS_MARGIN_PX = 10;
        /// @brief Baseline of the staleness readout, in pixels from the top.
        constexpr int STALENESS_Y_PX = 20;
        /// @brief Font scale used for the staleness readout.
        constexpr double STALENESS_FONT_SCALE = 0.5;
        /// @brief Stroke thickness used for the staleness readout.
        constexpr int STALENESS_THICKNESS = 1;
        /// @brief Decimal places shown for the detection age.
        constexpr int STALENESS_PRECISION = 2;

        /// @brief Encoding the overlay works in and republishes.
        const std::string IMAGE_ENCODING = "bgr8";
    }  // namespace

    DetectionOverlay::DetectionOverlay(const rclcpp::NodeOptions& options)
        : Node("detection_overlay", options)
    {
        _input_image_topic = declare_parameter<std::string>("input_image_topic", DEFAULT_INPUT_IMAGE_TOPIC);
        _output_image_topic = declare_parameter<std::string>("output_image_topic", DEFAULT_OUTPUT_IMAGE_TOPIC);
        _detection_topics = declare_parameter<std::vector<std::string>>("detection_topics", DEFAULT_DETECTION_TOPICS);
        _max_detection_age_s = declare_parameter<double>("max_detection_age_s", DEFAULT_MAX_DETECTION_AGE_S);
        _is_compressed_io = declare_parameter<bool>("compressed_io", DEFAULT_IS_COMPRESSED_IO);
        _should_show_staleness = declare_parameter<bool>("show_staleness", DEFAULT_SHOULD_SHOW_STALENESS);

        if (_is_compressed_io)
        {
            _compressed_image_subscription = create_subscription<sensor_msgs::msg::CompressedImage>(
                _input_image_topic + "/compressed", IMAGE_QOS_DEPTH,
                std::bind(&DetectionOverlay::_compressed_image_callback, this, std::placeholders::_1));
            _compressed_image_publisher = create_publisher<sensor_msgs::msg::CompressedImage>(
                _output_image_topic + "/compressed", IMAGE_QOS_DEPTH);
        }
        else
        {
            _image_subscription = create_subscription<sensor_msgs::msg::Image>(
                _input_image_topic, IMAGE_QOS_DEPTH,
                std::bind(&DetectionOverlay::_image_callback, this, std::placeholders::_1));
            _image_publisher = create_publisher<sensor_msgs::msg::Image>(
                _output_image_topic, IMAGE_QOS_DEPTH);
        }

        _detection_subscriptions.reserve(_detection_topics.size());
        for (const auto& topic : _detection_topics)
        {
            _detection_subscriptions.push_back(
                create_subscription<perseus_interfaces::msg::DetectionArray>(
                    topic, DETECTION_QOS_DEPTH,
                    [this, topic](const perseus_interfaces::msg::DetectionArray::SharedPtr msg)
                    { _detections_callback(topic, msg); }));

            RCLCPP_INFO(get_logger(), "Overlaying detections from: %s", topic.c_str());
        }

        RCLCPP_INFO(get_logger(), "DetectionOverlay ready — %s -> %s",
                    _input_image_topic.c_str(), _output_image_topic.c_str());
    }

    void DetectionOverlay::_detections_callback(
        const std::string& topic,
        const perseus_interfaces::msg::DetectionArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_detections_mutex);
        _latest_detections[topic] = *msg;
    }

    std::size_t DetectionOverlay::_draw_cached_detections(cv::Mat& frame, const rclcpp::Time& now)
    {
        // Copy under the lock, then draw outside it, so a slow render never stalls a
        // detector's callback.
        std::vector<perseus_interfaces::msg::DetectionArray> fresh_detections;
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            fresh_detections.reserve(_latest_detections.size());
            for (const auto& [topic, detections] : _latest_detections)
            {
                const rclcpp::Time stamp(detections.header.stamp, now.get_clock_type());
                const double age_s = (now - stamp).seconds();
                if (age_s < 0.0 || age_s > _max_detection_age_s)
                {
                    continue;
                }
                fresh_detections.push_back(detections);
            }
        }

        std::size_t drawn_count = 0;
        for (const auto& detections : fresh_detections)
        {
            draw_detections(frame, detections);
            drawn_count += detections.detections.size();
        }

        if (_should_show_staleness)
        {
            double oldest_age_s = 0.0;
            for (const auto& detections : fresh_detections)
            {
                const rclcpp::Time stamp(detections.header.stamp, now.get_clock_type());
                oldest_age_s = std::max(oldest_age_s, (now - stamp).seconds());
            }

            std::ostringstream staleness_stream;
            staleness_stream << "detections: " << drawn_count
                             << "  age: " << std::fixed << std::setprecision(STALENESS_PRECISION)
                             << oldest_age_s << "s";
            cv::putText(frame, staleness_stream.str(),
                        cv::Point(STALENESS_MARGIN_PX, STALENESS_Y_PX),
                        cv::FONT_HERSHEY_SIMPLEX, STALENESS_FONT_SCALE,
                        cv::Scalar(255, 255, 255), STALENESS_THICKNESS);
        }

        return drawn_count;
    }

    void DetectionOverlay::_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, IMAGE_ENCODING)->image;
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS,
                                  "cv_bridge exception: %s", e.what());
            return;
        }

        _draw_cached_detections(frame, msg->header.stamp);
        _image_publisher->publish(*cv_bridge::CvImage(msg->header, IMAGE_ENCODING, frame).toImageMsg());
    }

    void DetectionOverlay::_compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (frame.empty())
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS,
                                      "Failed to decode compressed image");
                return;
            }
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS,
                                  "OpenCV exception: %s", e.what());
            return;
        }

        _draw_cached_detections(frame, msg->header.stamp);

        sensor_msgs::msg::CompressedImage compressed_msg;
        compressed_msg.header = msg->header;
        compressed_msg.format = "jpeg";

        std::vector<uchar> buffer;
        const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
        cv::imencode(".jpg", frame, buffer, encode_params);
        compressed_msg.data = std::move(buffer);

        _compressed_image_publisher->publish(compressed_msg);
    }

}  // namespace perseus_vision

RCLCPP_COMPONENTS_REGISTER_NODE(perseus_vision::DetectionOverlay)
