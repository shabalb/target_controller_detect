#pragma once

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "target_controller_detect/msg/detection2_d.hpp"

namespace target_controller_detect {

struct ColorDetectionConfig {
  int grid_cols = 30;
  int grid_rows = 30;
  int min_area = 400;
  int blur_size = 27;
};

inline cv::Mat ensureBgrImage(const cv::Mat &input, const std::string &encoding) {
  cv::Mat bgr;
  if (encoding == "rgb8") {
    cv::cvtColor(input, bgr, cv::COLOR_RGB2BGR);
  } else if (encoding == "mono8") {
    cv::cvtColor(input, bgr, cv::COLOR_GRAY2BGR);
  } else {
    bgr = input.clone();
  }
  return bgr;
}

inline target_controller_detect::msg::Detection2D detectRedTarget(
  const cv::Mat &bgr,
  const std_msgs::msg::Header &header,
  const ColorDetectionConfig &config = {}) {
  target_controller_detect::msg::Detection2D detection;
  detection.header = header;
  detection.cell_x = -1;
  detection.cell_y = -1;

  if (bgr.empty()) {
    return detection;
  }

  cv::Mat blurred;
  const int blur_size = std::max(1, config.blur_size | 1);
  cv::GaussianBlur(bgr, blurred, cv::Size(blur_size, blur_size), 0);

  cv::Mat hsv;
  cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask1;
  cv::Mat mask2;
  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mask1);
  cv::inRange(hsv, cv::Scalar(170, 50, 50), cv::Scalar(180, 255, 255), mask2);
  mask = mask1 | mask2;

  const cv::Mat kernel =
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double best_score = 0.0;
  cv::Rect best_rect;
  for (const auto &contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < static_cast<double>(config.min_area)) {
      continue;
    }

    const cv::Rect rect = cv::boundingRect(contour);
    const double aspect =
      static_cast<double>(rect.width) / static_cast<double>(std::max(1, rect.height));
    if (aspect < 0.15 || aspect > 3.0) {
      continue;
    }

    std::vector<cv::Point> approx;
    const double perimeter = cv::arcLength(contour, true);
    cv::approxPolyDP(contour, approx, 0.02 * perimeter, true);
    if (static_cast<int>(approx.size()) >= 15) {
      continue;
    }

    const double fill = area / static_cast<double>(rect.area() + 1);
    const double score = area * fill;
    if (score > best_score) {
      best_score = score;
      best_rect = rect;
    }
  }

  if (best_score <= 0.0) {
    return detection;
  }

  detection.found = true;
  detection.x = best_rect.x;
  detection.y = best_rect.y;
  detection.width = best_rect.width;
  detection.height = best_rect.height;
  detection.center_x = best_rect.x + best_rect.width * 0.5f;
  detection.center_y = best_rect.y + best_rect.height * 0.5f;
  detection.score = static_cast<float>(best_score);

  const int cell_w = std::max(1, bgr.cols / std::max(1, config.grid_cols));
  const int cell_h = std::max(1, bgr.rows / std::max(1, config.grid_rows));
  detection.cell_x = std::clamp(
    static_cast<int>(detection.center_x / cell_w), 0, config.grid_cols - 1);
  detection.cell_y = std::clamp(
    static_cast<int>(detection.center_y / cell_h), 0, config.grid_rows - 1);
  return detection;
}

}  // namespace target_controller_detect
