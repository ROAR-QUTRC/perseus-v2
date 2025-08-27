#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>  // for CUDA EP append helper
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

namespace {
struct Detection {
  cv::Rect box;
  int cls;
  float score;
};

inline float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

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

// simple per-class NMS
std::vector<Detection> nms(const std::vector<Detection>& dets, float iou_thr) {
  std::vector<Detection> result;
  if (dets.empty()) return result;

  // group by class
  std::map<int, std::vector<Detection>> buckets;
  for (auto& d : dets) buckets[d.cls].push_back(d);

  for (auto& kv : buckets) {
    auto& v = kv.second;
    std::sort(v.begin(), v.end(), [](const Detection& a, const Detection& b){ return a.score > b.score; });
    std::vector<bool> removed(v.size(), false);
    for (size_t i = 0; i < v.size(); ++i) {
      if (removed[i]) continue;
      result.push_back(v[i]);
      for (size_t j = i + 1; j < v.size(); ++j) {
        if (removed[j]) continue;
        if (IoU(v[i].box, v[j].box) > iou_thr) removed[j] = true;
      }
    }
  }
  return result;
}

} // namespace

class CubeDetector : public rclcpp::Node {
public:
  CubeDetector() : rclcpp::Node("cube_detector") {
    RCLCPP_INFO(get_logger(), "Cube Detector Node (GPU + ONNX Runtime)");

    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rgbd_camera/image/cube_detection", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/rgbd_camera/image_raw", 10,
      std::bind(&CubeDetector::imageCb, this, std::placeholders::_1));

    // Load ONNX model (expects share/perseus_vision/yolo_model/best.onnx)
    const std::string model_path =
      ament_index_cpp::get_package_share_directory("perseus_vision") + "/yolo_model/best.onnx";
    RCLCPP_INFO(get_logger(), "Model path: %s", model_path.c_str());

    std::ifstream f(model_path);
    if (!f.good()) {
      RCLCPP_ERROR(get_logger(), "Model file not found.");
      return;
    }

    // ONNX Runtime env + session (CUDA EP)
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "cube-detector");
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(1);
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // ===== Enable CUDA Execution Provider =====
    // Device 0; change if you have multiple GPUs.
    OrtCUDAProviderOptions cuda_options{};
    cuda_options.device_id = 0;
    // optional tuning:
    cuda_options.arena_extend_strategy = 0; // kNextPowerOfTwo
    cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
    cuda_options.do_copy_in_default_stream = 1;

    Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(so, &cuda_options));

    session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), so);
    memory_info_ = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    // Cache I/O names (from your Colab check: input "images", output "output0")
    size_t num_in = session_->GetInputCount();
    size_t num_out = session_->GetOutputCount();
    input_names_.resize(num_in);
    output_names_.resize(num_out);

    Ort::AllocatorWithDefaultOptions allocator;
    for (size_t i = 0; i < num_in; ++i) {
      char* name = session_->GetInputName(i, allocator);
      input_names_[i] = name;
      allocator.Free(name);
    }
    for (size_t i = 0; i < num_out; ++i) {
      char* name = session_->GetOutputName(i, allocator);
      output_names_[i] = name;
      allocator.Free(name);
    }

    RCLCPP_INFO(get_logger(), "Inputs: %s", input_names_[0].c_str());
    RCLCPP_INFO(get_logger(), "Outputs: %s", output_names_[0].c_str());

    model_ok_ = true;
  }

private:
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!model_ok_) {
      // passthrough
      pub_->publish(*msg);
      return;
    }

    // Convert to OpenCV BGR
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    const int orig_w = bgr.cols;
    const int orig_h = bgr.rows;

    // Preprocess: BGR->RGB, resize to 640x640, normalize [0,1], CHW
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(640, 640), 0, 0, cv::INTER_LINEAR);

    resized.convertTo(resized, CV_32F, 1.0 / 255.0); // HWC float32
    std::vector<float> chw(1 * 3 * 640 * 640);
    // HWC -> CHW
    size_t c_stride = 640 * 640;
    std::vector<cv::Mat> ch(3);
    for (int c = 0; c < 3; ++c) {
      ch[c] = cv::Mat(640, 640, CV_32F, chw.data() + c * c_stride);
    }
    cv::split(resized, ch);

    // Create tensor
    std::array<int64_t, 4> input_shape{1, 3, 640, 640};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        *memory_info_, chw.data(), chw.size(), input_shape.data(), input_shape.size());

    // Run
    auto output_tensors = session_->Run(Ort::RunOptions{nullptr},
                                        input_names_.data(),
                                        &input_tensor, 1,
                                        output_names_.data(), 1);

    // Expect [1, 8, 8400]
    auto& out = output_tensors[0];
    auto info = out.GetTensorTypeAndShapeInfo();
    auto shape = info.GetShape();
    if (shape.size() != 3 || shape[0] != 1 || shape[1] != 8) {
      RCLCPP_WARN(get_logger(), "Unexpected output shape: [%ld,%ld,%ld]",
                  shape.size() > 0 ? shape[0] : -1,
                  shape.size() > 1 ? shape[1] : -1,
                  shape.size() > 2 ? shape[2] : -1);
    }

    const int num_feats = static_cast<int>(shape[1]);      // 8
    const int num_preds = static_cast<int>(shape[2]);      // 8400
    const float* p = out.GetTensorData<float>();

    // decode -> vector<Detection>
    constexpr float CONF_THR = 0.40f;  // tune as needed
    constexpr float NMS_IOU  = 0.50f;

    const float sx = static_cast<float>(orig_w) / 640.f;
    const float sy = static_cast<float>(orig_h) / 640.f;

    std::vector<Detection> dets;
    dets.reserve(256);

    for (int i = 0; i < num_preds; ++i) {
      // transposed access: feature k at offset k*num_preds + i
      float xc = p[0 * num_preds + i];
      float yc = p[1 * num_preds + i];
      float w  = p[2 * num_preds + i];
      float h  = p[3 * num_preds + i];
      float obj_logit = p[4 * num_preds + i];

      // classes start at feature index 5
      int best_cls = -1;
      float best_logit = -1e9f;
      for (int k = 5; k < num_feats; ++k) {
        float logit = p[k * num_preds + i];
        if (logit > best_logit) {
          best_logit = logit;
          best_cls = k - 5;
        }
      }

      float obj = sigmoid(obj_logit);
      float cls_conf = sigmoid(best_logit);
      float conf = obj * cls_conf;
      if (conf < CONF_THR) continue;

      // xywh → xyxy in model (640×640) space, then scale to original
      float x1 = (xc - w * 0.5f) * sx;
      float y1 = (yc - h * 0.5f) * sy;
      float x2 = (xc + w * 0.5f) * sx;
      float y2 = (yc + h * 0.5f) * sy;

      int ix1 = std::max(0, std::min(orig_w - 1, static_cast<int>(std::round(x1))));
      int iy1 = std::max(0, std::min(orig_h - 1, static_cast<int>(std::round(y1))));
      int ix2 = std::max(0, std::min(orig_w - 1, static_cast<int>(std::round(x2))));
      int iy2 = std::max(0, std::min(orig_h - 1, static_cast<int>(std::round(y2))));

      int bw = std::max(1, ix2 - ix1);
      int bh = std::max(1, iy2 - iy1);

      dets.push_back({cv::Rect(ix1, iy1, bw, bh), best_cls, conf});
    }

    // NMS
    auto final_dets = nms(dets, NMS_IOU);

    // Colors (BGR): 0-blue,1-green,2-red,3-white; default yellow
    auto color_for = [](int cls)->cv::Scalar{
      switch (cls) {
        case 0: return {255, 0, 0};
        case 1: return {0, 255, 0};
        case 2: return {0, 0, 255};
        case 3: return {255,255,255};
        default: return {0,255,255};
      }
    };

    // Draw
    for (const auto& d : final_dets) {
      cv::rectangle(bgr, d.box, color_for(d.cls), 2);
      char txt[64];
      std::snprintf(txt, sizeof(txt), "Class %d %.2f", d.cls, d.score);
      int base = 0;
      cv::Size t = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &base);
      cv::rectangle(bgr, {d.box.x, d.box.y - t.height - 6}, {d.box.x + t.width + 4, d.box.y}, color_for(d.cls), cv::FILLED);
      cv::putText(bgr, txt, {d.box.x + 2, d.box.y - 4}, cv::FONT_HERSHEY_SIMPLEX, 0.5, (d.cls==3? cv::Scalar(0,0,0): cv::Scalar(0,0,0)), 2);
    }

    // Publish annotated image
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", bgr).toImageMsg();
    pub_->publish(*out_msg);
  }

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  // ORT
  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  bool model_ok_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CubeDetector>());
  rclcpp::shutdown();
  return 0;
}
