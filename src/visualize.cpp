#include <opencv2/opencv.hpp>

void drawGrid(cv::Mat &img, const cv::Point &c, float px_per_m) {
  // окружности каждые 1м и оси
  for (int m = 1; m <= 10; ++m) {
    int r = static_cast<int>(m * px_per_m);
    cv::circle(img, c, r, cv::Scalar(40, 40, 40), 1, cv::LINE_AA);
  }
  cv::line(img, cv::Point(0, c.y), cv::Point(img.cols, c.y),
           cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
  cv::line(img, cv::Point(c.x, 0), cv::Point(c.x, img.rows),
           cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
}
