#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <onnxruntime_cxx_api.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <cmath>
#include <fstream>

namespace {
struct Detection {
  cv::Rect box;
  int cls;
  float score;
};

float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

float IoU(const cv::Rect& a, const cv::Rect& b) {
  int x1 = std::max(a.x, b.x);
  int y1 = std::max(a.y, b.y);
  int x2 = std::min(a.x + a.width,  b.x + b.width);
  int y2 = std::min(a.y + a.height, b.y + b.height);
  int inter = std::max(0, x2 - x1) * std::max(0, y2 - y1);
  int areaA = a.width * a.height;
  int areaB = b.width * b.height;
  int uni = areaA + areaB - inter;
  return uni > 0 ? static_cast<float>(inter) / static_cast<float>(uni) : 0.f;
}

std::vector<Detection> nms(const std::vector<Detection>& dets, float iou_thr) {
  std::vector<Detection> result;
  std::map<int, std::vector<Detection>> buckets;
  for (auto& d : dets) buckets[d.cls].push_back(d);

  for (auto& [cls, vec] : buckets) {
    std::sort(vec.begin(), vec.end(), [](const Detection& a, const Detection& b) {
      return a.score > b.score;
    });
    std::vector<bool> removed(vec.size(), false);
    for (size_t i = 0; i < vec.size(); ++i) {
      if (removed[i]) continue;
      result.push_back(vec[i]);
      for (size_t j = i + 1; j < vec.size(); ++j) {
        if (!removed[j] && IoU(vec[i].box, vec[j].box) > iou_thr) removed[j] = true;
      }
    }
  }
  return result;
}
}

class CubeDetector : public rclcpp::Node {
public:
  CubeDetector() : Node("cube_detector") {
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/rgbd_camera/image/cube_detection", 10);
    pub_pose_  = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cube_detector/cube_pose", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/rgbd_camera/image_raw", 10,
      std::bind(&CubeDetector::imageCb, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string model_path = ament_index_cpp::get_package_share_directory("perseus_vision") + "/yolo_model/best.onnx";
    RCLCPP_INFO(get_logger(), "Loading model: %s", model_path.c_str());

    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "yolo");
    Ort::SessionOptions so;
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), so);
    memory_info_ = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    Ort::AllocatorWithDefaultOptions allocator;
    size_t in_count = session_->GetInputCount();
    input_names_.resize(in_count);
    for (size_t i = 0; i < in_count; ++i)
      input_names_[i] = session_->GetInputNameAllocated(i, allocator).get();

    size_t out_count = session_->GetOutputCount();
    output_names_.resize(out_count);
    for (size_t i = 0; i < out_count; ++i)
      output_names_[i] = session_->GetOutputNameAllocated(i, allocator).get();

    model_loaded_ = true;
  }

private:
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!model_loaded_) return;

    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to convert image.");
      return;
    }

    int ow = bgr.cols, oh = bgr.rows;

    cv::Mat input;
    cv::cvtColor(bgr, input, cv::COLOR_BGR2RGB);
    cv::resize(input, input, cv::Size(640, 640));
    input.convertTo(input, CV_32F, 1.0 / 255.0);

    std::vector<float> chw(3 * 640 * 640);
    for (int c = 0; c < 3; ++c)
      for (int y = 0; y < 640; ++y)
        for (int x = 0; x < 640; ++x)
          chw[c * 640 * 640 + y * 640 + x] = input.at<cv::Vec3f>(y, x)[c];

    std::array<int64_t, 4> shape{1, 3, 640, 640};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(*memory_info_, chw.data(), chw.size(), shape.data(), shape.size());

    auto output = session_->Run(Ort::RunOptions{nullptr},
                                input_names_cstr(), &input_tensor, 1,
                                output_names_cstr(), 1);

    const float* out = output[0].GetTensorData<float>();
    auto dims = output[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_feats = dims[1], num_preds = dims[2];

    float scale_x = static_cast<float>(ow) / 640.0f;
    float scale_y = static_cast<float>(oh) / 640.0f;

    std::vector<Detection> detections;
    for (int i = 0; i < num_preds; ++i) {
      float xc = out[0 * num_preds + i];
      float yc = out[1 * num_preds + i];
      float w  = out[2 * num_preds + i];
      float h  = out[3 * num_preds + i];
      float obj = sigmoid(out[4 * num_preds + i]);

      int cls = -1;
      float best = -1e9;
      for (int j = 5; j < num_feats; ++j) {
        float val = out[j * num_preds + i];
        if (val > best) {
          best = val;
          cls = j - 5;
        }
      }

      float conf = obj * sigmoid(best);
      if (conf < 0.3f) continue;

      int x1 = (xc - w/2) * scale_x;
      int y1 = (yc - h/2) * scale_y;
      int x2 = (xc + w/2) * scale_x;
      int y2 = (yc + h/2) * scale_y;
      detections.push_back({cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), cls, conf});
    }

    auto final_dets = nms(detections, 0.5f);

    for (const auto& d : final_dets) {
      cv::Point center(d.box.x + d.box.width/2, d.box.y + d.box.height/2);

      // Simulated depth (you can replace this with real depth value)
      float Z = 1.0f;  // 1 meter
      float fx = 525.0f, fy = 525.0f, cx = ow / 2.0f, cy = oh / 2.0f;
      float X = (center.x - cx) * Z / fx;
      float Y = (center.y - cy) * Z / fy;

      geometry_msgs::msg::PoseStamped cam_pose;
      cam_pose.header = msg->header;
      cam_pose.pose.position.x = X;
      cam_pose.pose.position.y = Y;
      cam_pose.pose.position.z = Z;
      cam_pose.pose.orientation.w = 1.0;

      try {
        geometry_msgs::msg::PoseStamped odom_pose = tf_buffer_->transform(cam_pose, "odom", tf2::durationFromSec(0.1));
        pub_pose_->publish(odom_pose);
        RCLCPP_INFO(get_logger(), "Cube in odom: [%.2f %.2f %.2f]", 
                    odom_pose.pose.position.x,
                    odom_pose.pose.position.y,
                    odom_pose.pose.position.z);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF2 transform failed: %s", ex.what());
      }

      cv::rectangle(bgr, d.box, {0, 255, 0}, 2);
    }

    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", bgr).toImageMsg();
    pub_image_->publish(*out_msg);
  }

  const char** input_names_cstr() {
    input_cstr_.clear();
    for (auto& s : input_names_) input_cstr_.push_back(s.c_str());
    return input_cstr_.data();
  }

  const char** output_names_cstr() {
    output_cstr_.clear();
    for (auto& s : output_names_) output_cstr_.push_back(s.c_str());
    return output_cstr_.data();
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  std::vector<std::string> input_names_, output_names_;
  std::vector<const char*> input_cstr_, output_cstr_;
  bool model_loaded_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CubeDetector>());
  rclcpp::shutdown();
  return 0;
}
