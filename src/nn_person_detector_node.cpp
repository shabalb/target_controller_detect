#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "target_controller_detect/detector_utils.hpp"
#include "target_controller_detect/msg/detection2_d.hpp"
#include "target_controller_detect/shm_image_ring.hpp"

namespace {

struct LetterboxResult {
  cv::Mat image;
  float scale = 1.0f;
  int pad_x = 0;
  int pad_y = 0;
};

LetterboxResult makeLetterbox(const cv::Mat &bgr, int dst_w, int dst_h) {
  LetterboxResult result;
  result.image = cv::Mat(dst_h, dst_w, CV_8UC3, cv::Scalar(114, 114, 114));
  if (bgr.empty()) {
    return result;
  }

  const float scale = std::min(
    static_cast<float>(dst_w) / static_cast<float>(bgr.cols),
    static_cast<float>(dst_h) / static_cast<float>(bgr.rows));
  const int resized_w = std::max(1, static_cast<int>(std::round(bgr.cols * scale)));
  const int resized_h = std::max(1, static_cast<int>(std::round(bgr.rows * scale)));

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(resized_w, resized_h));

  const int pad_x = (dst_w - resized_w) / 2;
  const int pad_y = (dst_h - resized_h) / 2;
  resized.copyTo(result.image(cv::Rect(pad_x, pad_y, resized_w, resized_h)));

  result.scale = scale;
  result.pad_x = pad_x;
  result.pad_y = pad_y;
  return result;
}

cv::Mat normalizeDetections(const cv::Mat &output) {
  if (output.empty()) {
    return {};
  }

  if (output.dims == 3) {
    const int d1 = output.size[1];
    const int d2 = output.size[2];
    if (d1 <= 0 || d2 <= 0) {
      return {};
    }

    cv::Mat tmp(d1, d2, CV_32F, const_cast<float *>(output.ptr<float>()));
    cv::Mat normalized;
    // Typical YOLOv8 ONNX output is [1, 84, 8400], transpose to [8400, 84].
    if (d1 < d2 && d1 <= 256) {
      cv::transpose(tmp, normalized);
      return normalized;
    }

    return tmp.clone();
  }

  if (output.dims == 2) {
    cv::Mat normalized = output;
    if (normalized.type() != CV_32F) {
      normalized.convertTo(normalized, CV_32F);
    }
    if (normalized.rows < normalized.cols && normalized.rows <= 256) {
      cv::transpose(normalized, normalized);
    }
    return normalized;
  }

  return {};
}

int parseBackend(const std::string &value) {
  if (value == "opencv") {
    return cv::dnn::DNN_BACKEND_OPENCV;
  }
  if (value == "cuda") {
    return cv::dnn::DNN_BACKEND_CUDA;
  }
  return cv::dnn::DNN_BACKEND_DEFAULT;
}

int parseTarget(const std::string &value) {
  if (value == "cpu") {
    return cv::dnn::DNN_TARGET_CPU;
  }
  if (value == "opencl") {
    return cv::dnn::DNN_TARGET_OPENCL;
  }
  if (value == "opencl_fp16") {
    return cv::dnn::DNN_TARGET_OPENCL_FP16;
  }
  if (value == "cuda") {
    return cv::dnn::DNN_TARGET_CUDA;
  }
  if (value == "cuda_fp16") {
    return cv::dnn::DNN_TARGET_CUDA_FP16;
  }
  return cv::dnn::DNN_TARGET_CPU;
}

cv::Mat shmViewToBgr(const target_controller_detect::ShmImageView &view) {
  const int width = static_cast<int>(view.width);
  const int height = static_cast<int>(view.height);
  if (width <= 0 || height <= 0) {
    return {};
  }

  if (view.encoding == "bgr8") {
    if (view.step < view.width * 3U) {
      throw std::runtime_error("bgr8 SHM step is too small");
    }
    cv::Mat src(height, width, CV_8UC3, const_cast<std::uint8_t *>(view.data), view.step);
    return src.clone();
  }

  if (view.encoding == "rgb8") {
    if (view.step < view.width * 3U) {
      throw std::runtime_error("rgb8 SHM step is too small");
    }
    cv::Mat src(height, width, CV_8UC3, const_cast<std::uint8_t *>(view.data), view.step);
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }

  if (view.encoding == "mono8") {
    if (view.step < view.width) {
      throw std::runtime_error("mono8 SHM step is too small");
    }
    cv::Mat src(height, width, CV_8UC1, const_cast<std::uint8_t *>(view.data), view.step);
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }

  if (view.encoding == "nv12") {
    const auto expected = static_cast<std::uint32_t>(view.width * view.height * 3U / 2U);
    if (view.data_size < expected) {
      throw std::runtime_error("nv12 SHM data_size is too small");
    }
    cv::Mat nv12(height * 3 / 2, width, CV_8UC1, const_cast<std::uint8_t *>(view.data));
    cv::Mat bgr;
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    return bgr;
  }

  throw std::runtime_error("Unsupported RGB SHM encoding: " + view.encoding);
}

}  // namespace

class NnPersonDetectorNode : public rclcpp::Node {
public:
  NnPersonDetectorNode() : Node("nn_person_detector_node") {
    input_source_ = declare_parameter<std::string>("input_source", "ros");
    shm_name_ = declare_parameter<std::string>("shm_name", "oak_rgb");
    const auto camera_topic = declare_parameter<std::string>("camera_topic", "/camera/image");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/target/detection2d");
    model_path_ = declare_parameter<std::string>("model_path", "");

    input_width_ = declare_parameter<int>("input_width", 640);
    input_height_ = declare_parameter<int>("input_height", 640);
    conf_threshold_ = declare_parameter<double>("conf_threshold", 0.40);
    nms_threshold_ = declare_parameter<double>("nms_threshold", 0.45);
    process_fps_ = declare_parameter<double>("process_fps", 15.0);
    grid_cols_ = declare_parameter<int>("grid_cols", 30);
    grid_rows_ = declare_parameter<int>("grid_rows", 30);
    show_windows_ = declare_parameter<bool>("show_windows", false);

    const auto backend = declare_parameter<std::string>("dnn_backend", "opencv");
    const auto target = declare_parameter<std::string>("dnn_target", "cpu");

    loadModel(backend, target);

    rclcpp::QoS qos(rclcpp::KeepLast(20));
    qos.best_effort();

    detection_pub_ = create_publisher<target_controller_detect::msg::Detection2D>(detection_topic_, 10);
    if (input_source_ == "shm") {
      const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, process_fps_));
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&NnPersonDetectorNode::onTimer, this));
    } else {
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        camera_topic, qos, std::bind(&NnPersonDetectorNode::onImage, this, std::placeholders::_1));
    }

    if (show_windows_) {
      cv::namedWindow("nn_detector_camera", cv::WINDOW_NORMAL);
    }

    RCLCPP_INFO(get_logger(), "Input source: %s", input_source_.c_str());
    RCLCPP_INFO(get_logger(), "Camera topic: %s", camera_topic.c_str());
    RCLCPP_INFO(get_logger(), "RGB SHM: %s", shm_name_.c_str());
    RCLCPP_INFO(get_logger(), "Detection topic: %s", detection_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Model path: %s", model_path_.c_str());
  }

  ~NnPersonDetectorNode() override {
    if (show_windows_) {
      cv::destroyWindow("nn_detector_camera");
    }
  }

private:
  bool ensureReader() {
    if (reader_) {
      return true;
    }

    try {
      reader_ = std::make_unique<target_controller_detect::ShmImageRingReader>(shm_name_);
      RCLCPP_INFO(get_logger(), "Opened RGB shared memory: %s", shm_name_.c_str());
      return true;
    } catch (const std::exception &exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for RGB SHM '%s': %s",
        shm_name_.c_str(), exc.what());
      return false;
    }
  }

  void loadModel(const std::string &backend, const std::string &target) {
    if (model_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'model_path' is empty. Detector will publish empty detections.");
      return;
    }

    try {
      net_ = cv::dnn::readNet(model_path_);
      net_.setPreferableBackend(parseBackend(backend));
      net_.setPreferableTarget(parseTarget(target));
      output_names_ = net_.getUnconnectedOutLayersNames();
      model_ready_ = true;
      RCLCPP_INFO(
        get_logger(), "Loaded model (%s), backend=%s, target=%s", model_path_.c_str(),
        backend.c_str(), target.c_str());
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to load model '%s': %s", model_path_.c_str(), e.what());
      model_ready_ = false;
    }
  }

  void publishDetection(
    const cv::Mat &bgr,
    const std_msgs::msg::Header &header) {
    auto detection = infer(bgr, header);
    detection_pub_->publish(detection);

    if (show_windows_) {
      cv::Mat debug = bgr.clone();
      if (detection.found) {
        cv::rectangle(
          debug, cv::Rect(detection.x, detection.y, detection.width, detection.height),
          cv::Scalar(0, 255, 0), 2);
        cv::putText(
          debug, "person " + std::to_string(detection.score),
          cv::Point(detection.x, std::max(0, detection.y - 8)), cv::FONT_HERSHEY_SIMPLEX, 0.6,
          cv::Scalar(0, 255, 0), 2);
      }

      cv::putText(
        debug, "nn detector", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.7,
        cv::Scalar(0, 255, 255), 2);
      cv::imshow("nn_detector_camera", debug);
      cv::waitKey(1);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "NN detection found=%d center=(%.1f, %.1f) score=%.3f",
      detection.found, detection.center_x, detection.center_y, detection.score);
  }

  void onTimer() {
    if (!ensureReader()) {
      return;
    }

    target_controller_detect::ShmImageView view;
    if (!reader_->latest(view) || view.seq == last_seq_) {
      return;
    }
    last_seq_ = view.seq;

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "oak_rgb_camera_optical_frame";

    target_controller_detect::msg::Detection2D empty_detection;
    empty_detection.header = header;
    empty_detection.cell_x = -1;
    empty_detection.cell_y = -1;

    try {
      cv::Mat bgr = shmViewToBgr(view);
      if (bgr.empty()) {
        detection_pub_->publish(empty_detection);
        return;
      }
      publishDetection(bgr, header);
    } catch (const cv::Exception &exc) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", exc.what());
      detection_pub_->publish(empty_detection);
    } catch (const std::exception &exc) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "SHM image error: %s", exc.what());
      detection_pub_->publish(empty_detection);
    }
  }

  target_controller_detect::msg::Detection2D infer(
    const cv::Mat &bgr,
    const std_msgs::msg::Header &header) {
    target_controller_detect::msg::Detection2D detection;
    detection.header = header;
    detection.cell_x = -1;
    detection.cell_y = -1;

    if (!model_ready_ || bgr.empty()) {
      return detection;
    }

    auto lb = makeLetterbox(bgr, input_width_, input_height_);

    cv::Mat blob;
    cv::dnn::blobFromImage(
      lb.image, blob, 1.0 / 255.0, cv::Size(input_width_, input_height_), cv::Scalar(), true, false,
      CV_32F);

    net_.setInput(blob);

    std::vector<cv::Mat> outputs;
    net_.forward(outputs, output_names_);
    if (outputs.empty()) {
      return detection;
    }

    const cv::Mat det = normalizeDetections(outputs.front());
    if (det.empty() || det.cols < 5) {
      return detection;
    }

    const int person_col = 4;  // COCO class 0 (person) in YOLOv8 => column 4 + 0.
    if (person_col >= det.cols) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Detector output has unexpected shape (%d x %d)", det.rows, det.cols);
      return detection;
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    boxes.reserve(static_cast<size_t>(det.rows));
    scores.reserve(static_cast<size_t>(det.rows));

    for (int i = 0; i < det.rows; ++i) {
      const float *row = det.ptr<float>(i);
      const float score = row[person_col];
      if (!std::isfinite(score) || score < static_cast<float>(conf_threshold_)) {
        continue;
      }

      const float cx = row[0];
      const float cy = row[1];
      const float w = row[2];
      const float h = row[3];
      if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h)) {
        continue;
      }

      const float x1 = (cx - 0.5f * w - static_cast<float>(lb.pad_x)) / lb.scale;
      const float y1 = (cy - 0.5f * h - static_cast<float>(lb.pad_y)) / lb.scale;
      const float x2 = (cx + 0.5f * w - static_cast<float>(lb.pad_x)) / lb.scale;
      const float y2 = (cy + 0.5f * h - static_cast<float>(lb.pad_y)) / lb.scale;

      const int left = std::clamp(static_cast<int>(std::round(x1)), 0, bgr.cols - 1);
      const int top = std::clamp(static_cast<int>(std::round(y1)), 0, bgr.rows - 1);
      const int right = std::clamp(static_cast<int>(std::round(x2)), 0, bgr.cols - 1);
      const int bottom = std::clamp(static_cast<int>(std::round(y2)), 0, bgr.rows - 1);

      const int box_w = right - left;
      const int box_h = bottom - top;
      if (box_w < 2 || box_h < 2) {
        continue;
      }

      boxes.emplace_back(left, top, box_w, box_h);
      scores.push_back(score);
    }

    if (boxes.empty()) {
      return detection;
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(
      boxes, scores, static_cast<float>(conf_threshold_), static_cast<float>(nms_threshold_), keep);
    if (keep.empty()) {
      return detection;
    }

    int best_idx = keep.front();
    float best_score = scores[best_idx];
    for (const int idx : keep) {
      if (scores[idx] > best_score) {
        best_score = scores[idx];
        best_idx = idx;
      }
    }

    const auto &best = boxes[best_idx];
    detection.found = true;
    detection.x = best.x;
    detection.y = best.y;
    detection.width = best.width;
    detection.height = best.height;
    detection.center_x = static_cast<float>(best.x) + static_cast<float>(best.width) * 0.5f;
    detection.center_y = static_cast<float>(best.y) + static_cast<float>(best.height) * 0.5f;
    detection.score = best_score;

    const int cell_w = std::max(1, bgr.cols / std::max(1, grid_cols_));
    const int cell_h = std::max(1, bgr.rows / std::max(1, grid_rows_));
    detection.cell_x = std::clamp(
      static_cast<int>(detection.center_x / static_cast<float>(cell_w)), 0, grid_cols_ - 1);
    detection.cell_y = std::clamp(
      static_cast<int>(detection.center_y / static_cast<float>(cell_h)), 0, grid_rows_ - 1);

    return detection;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    target_controller_detect::msg::Detection2D empty_detection;
    empty_detection.header = msg->header;
    empty_detection.cell_x = -1;
    empty_detection.cell_y = -1;

    try {
      cv_bridge::CvImageConstPtr cv_ptr;
      if (msg->encoding == sensor_msgs::image_encodings::RGB8 || msg->encoding == "rgb8") {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
      } else if (
        msg->encoding == sensor_msgs::image_encodings::BGR8 || msg->encoding == "bgr8")
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      } else if (
        msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == "mono8")
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      } else {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      }

      cv::Mat bgr = target_controller_detect::ensureBgrImage(cv_ptr->image, msg->encoding);
      publishDetection(bgr, msg->header);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      detection_pub_->publish(empty_detection);
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
      detection_pub_->publish(empty_detection);
    }
  }

  std::string input_source_;
  std::string shm_name_;
  std::string detection_topic_;
  std::string model_path_;

  int input_width_{640};
  int input_height_{640};
  int grid_cols_{30};
  int grid_rows_{30};
  double process_fps_{15.0};
  double conf_threshold_{0.40};
  double nms_threshold_{0.45};
  bool show_windows_{false};
  std::uint64_t last_seq_{0};

  bool model_ready_{false};
  cv::dnn::Net net_;
  std::vector<std::string> output_names_;
  std::unique_ptr<target_controller_detect::ShmImageRingReader> reader_;

  rclcpp::Publisher<target_controller_detect::msg::Detection2D>::SharedPtr detection_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NnPersonDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
