#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

//#include "kalman.cpp"
#include "visualize.cpp"
//#include "structs.cpp"

#define QT true
#define CONTROL true

////////////////// параметры
//float CAMERA_WIDTH = 640;
//float CAMERA_FOV = 1.047;
float CAMERA_WIDTH = 640;
float CAMERA_FOV = 1.466;
char const *CAMERA_TOPIC = "/camera/image";
char const *DEPTH_TOPIC = "/oak/stereo/depth";
char const *POINT_CLOUD_TOPIC = "/oak/stereo/points";
char const *TWIST_TOPIC = "/cmd_vel";

float Kd = 0.8f; // по расстоянию
float Ka = 1.5f; // по углу

float max_linear = 0.3f; // скорость
float max_angular = 0.1f;

float desired_distance = 0.7f; // удерживаемое расстояние
float dist_deadband = 0.25f;   // мертвая зона расстояния
float angle_align_deadband = 0.05f;   // грубое центрирование (~3 градуса)
float angle_follow_deadband = 0.02f;  // мелкая подстройка при движении
/////////////////

//disabled filter delay time

//////////////////////////////////

struct Detection {
  bool found = false;
  cv::Rect bbox;
  cv::Point2f center;
  int cell_x = -1;
  int cell_y = -1;
};

struct CloudPoint {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float range = 0.0f;
  float angle = 0.0f;
};

struct MotionCommand {
  float linear = 0.0f;  // линейная скорость
  float angular = 0.0f; // угловая скорость
};

struct TargetState {  // состояние цели
  bool valid = false; // найдена ли цель
  bool lost = true;
  float distance = 0.0f; // расстояние до цели
  float angle = 0.0f;    // угол до цели
  float rel_x = 0.0f; // вперед/назад относительно робота
  float rel_y = 0.0f; // вбок относительно робота
};

// режимы преследования
enum class FollowMode { SEARCH, ALIGN, FOLLOW, STOP, LOST };



class ImageViewer : public rclcpp::Node {
public:
  ImageViewer() : Node("image_viewer") {
    rclcpp::QoS qos(rclcpp::KeepLast(20));
    qos.best_effort();
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      CAMERA_TOPIC, qos,
      std::bind(&ImageViewer::onImage, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      DEPTH_TOPIC, qos,
      std::bind(&ImageViewer::onDepthImage, this, std::placeholders::_1));
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      POINT_CLOUD_TOPIC, qos,
      std::bind(&ImageViewer::onPointCloud, this, std::placeholders::_1));
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(TWIST_TOPIC, 10);
    cmd_vel_raw = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 10);

#if QT == true
    cv::namedWindow("camera", cv::WINDOW_NORMAL);
    cv::namedWindow("depth", cv::WINDOW_NORMAL);
    cv::namedWindow(win_, cv::WINDOW_AUTOSIZE);
#endif

    RCLCPP_INFO(get_logger(), "Subscribed camera: %s", CAMERA_TOPIC);
    RCLCPP_INFO(get_logger(), "Subscribed depth: %s", DEPTH_TOPIC);
    RCLCPP_INFO(get_logger(), "Subscribed cloud: %s", POINT_CLOUD_TOPIC);
  }

  ~ImageViewer() override { cv::destroyAllWindows(); }

private:
  size_t n_ = 0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw; 
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  //sensor_msgs::msg::Image::ConstSharedPtr last_image_;
  Detection last_detection;
  rclcpp::Time last_image_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_detection_stamp_{0, 0, RCL_ROS_TIME};
  int image_width_ = 640;
  int image_height_ = 480;
  double detection_hold_sec_ = 0.6;

  float median(std::vector<float> &values) const {
    if (values.empty()) {
      return 0.0f;
    }
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    float med = values[mid];
    if (values.size() % 2 == 0) {
      const auto max_it = std::max_element(values.begin(), values.begin() + mid);
      med = 0.5f * (med + *max_it);
    }
    return med;
  }

  void showDepthDebug(const cv::Mat &depth_meters) {
#if QT == true
    if (depth_meters.empty()) {
      return;
    }
    cv::Mat finite_mask = (depth_meters > 0.05f) & (depth_meters < 20.0f);
    double min_v = 0.0;
    double max_v = 0.0;
    cv::minMaxLoc(depth_meters, &min_v, &max_v, nullptr, nullptr, finite_mask);
    if (max_v <= min_v) {
      max_v = min_v + 1.0;
    }

    cv::Mat normalized(depth_meters.size(), CV_8UC1, cv::Scalar(0));
    depth_meters.convertTo(normalized, CV_8UC1, 255.0 / (max_v - min_v),
                           -min_v * 255.0 / (max_v - min_v));
    normalized.setTo(0, ~finite_mask);

    cv::Mat depth_color;
    cv::applyColorMap(normalized, depth_color, cv::COLORMAP_TURBO);
    cv::putText(depth_color, "depth (m)",
                cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 2);
    cv::imshow("depth", depth_color);
    cv::waitKey(1);
#endif
  }

  void processTogether(const std::vector<CloudPoint> &object_points,
                       const Detection &camera_data) {
    auto t0 = this->now();
    TargetState state = score(object_points, camera_data);

    RCLCPP_INFO(this->get_logger(),
                "TargetState: distance: %f, angle: %f, valid: %d",
                state.distance, state.angle, state.valid);
    FollowMode mode = decide(state);
    //RCLCPP_INFO(this->get_logger(), "статус %d", (int)mode);
    MotionCommand cmd = compute(state, mode);
    RCLCPP_INFO(this->get_logger(), "поворот %f", cmd.angular);
    RCLCPP_INFO(this->get_logger(), "линейная скорость %f", cmd.linear);
#if CONTROL == true
    sendCommand(cmd);
#endif
    // RCLCPP_INFO(this->get_logger(), "send out");

    // RCLCPP_INFO(this->get_logger(),
    //             "Получили синхронизированную пару и обработали её");
    auto dtf = (this->now() - t0).seconds();
    RCLCPP_INFO(this->get_logger(), "processTogether took %.3f s", dtf);
  }

  ///////////////////////// формирование команд

  FollowMode decide(const TargetState &target) const {
    if (!target.valid)
      return FollowMode::LOST;

    if (std::fabs(target.angle) > angle_align_deadband)
      return FollowMode::ALIGN;

    float dist_error = target.distance - desired_distance;

    if (std::fabs(dist_error) < dist_deadband &&
        std::fabs(target.angle) < angle_follow_deadband)
      return FollowMode::STOP;

    return FollowMode::FOLLOW;
  }

  MotionCommand compute(const TargetState &target, FollowMode mode) const {
    MotionCommand cmd;
    //if (!target.valid) {
      // цель потеряна
      //cmd.linear = 0.0f;
      //cmd.angular = 0.0f;
      //return cmd;
    //}

    float dist_error = target.distance - desired_distance;
    float angle_error = target.angle;

    switch (mode) {
    case FollowMode::ALIGN:
      cmd.linear = 0.0f;
      cmd.angular = -std::clamp(Ka * angle_error, -max_angular, max_angular);
      RCLCPP_INFO(this->get_logger(), "ALIGN");
      break;

    case FollowMode::FOLLOW:
      if (std::fabs(dist_error) >= dist_deadband)
        cmd.linear = std::clamp(Kd * dist_error, -max_linear, max_linear);

      if (std::fabs(angle_error) >= angle_follow_deadband)
        cmd.angular = -std::clamp(Ka * angle_error, -max_angular, max_angular);
      break;

    case FollowMode::STOP:
      cmd.linear = 0.0f;
      cmd.angular = 0.0f;
      break;

    case FollowMode::SEARCH:
    case FollowMode::LOST:
      cmd.linear = 0.0f;
      cmd.angular = 0.07f; 
      RCLCPP_INFO(this->get_logger(), "LOST");
      break;
    }

    return cmd;
  }

  TargetState score(const std::vector<CloudPoint> &object_points,
                    const Detection &camera_data) {
    TargetState state;

    if (!camera_data.found || object_points.empty()) {
      RCLCPP_INFO(this->get_logger(),
                  "score invalid: camera_found=%d object_points.empty()=%d",
                  camera_data.found, object_points.empty());
      state.valid = false;
      state.lost = true;
      return state;
    }

    state.valid = true;
    state.lost = false;

    std::vector<float> depth_values;
    depth_values.reserve(object_points.size());

    for (const auto &p : object_points) {
      if (!std::isfinite(p.range))
        continue;
      if (p.range < 0.1f || p.range > 20.0f)
        continue;
      depth_values.push_back(p.range);
    }

    if (depth_values.empty()) {
      state.valid = false;
      state.lost = true;
      return state;
    }

    state.distance = median(depth_values);

    // Use the RGB detection center for heading control. This keeps the target
    // centered in the camera frame even if the depth cloud is slightly shifted
    // relative to the color image.
    const float image_cx = 0.5f * static_cast<float>(image_width_);
    const float fx_pixels =
        static_cast<float>(image_width_) /
        (2.0f * std::tan(CAMERA_FOV / 2.0f));
    const float pixel_offset = camera_data.center.x - image_cx;
    state.angle = std::atan2(pixel_offset, fx_pixels);

    RCLCPP_INFO(this->get_logger(),
                "score: depth_samples=%zu pixel_offset=%f angle=%f",
                depth_values.size(), pixel_offset, state.angle);

    return state;
  }

  void sendCommand(MotionCommand cmd) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = cmd.linear;
    msg.angular.z = cmd.angular;
    cmd_pub_->publish(msg);
    //cmd_vel_raw->publish(msg);
  }

  ////////////////////////////////////////////////////////////////

  Detection onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    Detection defDetect;
    auto t0 = this->now();
    n_++;
    if (n_ % 30 == 0) {
      RCLCPP_INFO(get_logger(), "frames=%zu stamp=%u.%u encoding=%s", n_,
                  msg->header.stamp.sec, msg->header.stamp.nanosec,
                  msg->encoding.c_str());
    }
    try {

      cv::Mat bgr;
      if (msg->encoding == sensor_msgs::image_encodings::RGB8 ||
          msg->encoding == "rgb8") {
        auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == sensor_msgs::image_encodings::BGR8 ||
                 msg->encoding == "bgr8") {
        auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        bgr = cv_ptr->image.clone();
      } else if (msg->encoding == sensor_msgs::image_encodings::MONO8 ||
                 msg->encoding == "mono8") {
        auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_GRAY2BGR);
      } else {
        auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        bgr = cv_ptr->image.clone();
      }

      image_width_ = bgr.cols;
      image_height_ = bgr.rows;

      auto det = detectRedSquareAndCell(bgr, 30, 30);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "maxAspect %.1f",
                           this->maxAspect);
      if (det.found) {
        cv::rectangle(bgr, det.bbox, cv::Scalar(0, 255, 0), 2);
        cv::circle(bgr, det.center, 3, cv::Scalar(255, 255, 255), -1);
        std::string txt = "cell=(" + std::to_string(det.cell_x) + "," +
                          std::to_string(det.cell_y) + ")";
        cv::putText(bgr, txt,
                    cv::Point(det.bbox.x, std::max(0, det.bbox.y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "Red square at px=(%.1f,%.1f) cell=(%d,%d)",
                             det.center.x, det.center.y, det.cell_x,
                             det.cell_y);
        last_detection = det;
        last_detection_stamp_ = rclcpp::Time(msg->header.stamp);
      } else {
        const rclcpp::Time image_t(msg->header.stamp);
        const double age = std::abs((image_t - last_detection_stamp_).seconds());
        if (age > detection_hold_sec_) {
          last_detection = defDetect;
        }
      }

#if QT == true
      cv::putText(bgr, std::string("rgb: ") + CAMERA_TOPIC,
                  cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 255), 2);
      cv::imshow("camera", bgr);
      cv::waitKey(1);
#endif
      last_image_stamp_ = rclcpp::Time(msg->header.stamp);
      auto dtf = (this->now() - t0).seconds();
      RCLCPP_INFO(this->get_logger(), "onImage took %.3f s", dtf);
      return last_detection;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return defDetect;
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
      return defDetect;
    }
    return defDetect;
  }

  void onDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      cv::Mat depth_meters;
      if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
          msg->encoding == "32FC1") {
        auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_meters = cv_ptr->image;
      } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                 msg->encoding == "16UC1") {
        auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr->image.convertTo(depth_meters, CV_32FC1, 0.001);
      } else {
        auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        cv_ptr->image.convertTo(depth_meters, CV_32FC1, 1.0 / 255.0);
      }
      showDepthDebug(depth_meters);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Depth frame: %ux%u encoding=%s",
                           msg->width, msg->height, msg->encoding.c_str());
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "depth cv_bridge exception: %s", e.what());
    }
  }

  Detection detectRedSquareAndCell(const cv::Mat &bgr, int grid_cols,
                                   int grid_rows) {
    Detection d;
    if (bgr.empty()){
      std::cout<< "image is empty in detect"<<std::endl;
      return d;
    }
    cv::Mat blurred;
    cv::GaussianBlur(bgr, blurred, cv::Size(27, 27), 0);
    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    // Красный: два диапазона Hue
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 50, 50), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;
    #if QT == true
    //cv::imshow("mask1", mask1);
    //cv::imshow("mask2", mask2);
    cv::imshow("mask", mask);
    #endif
    // Убираем шум
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    
    cv::Mat contour_vis = bgr.clone();
    cv::drawContours(contour_vis, contours, -1, cv::Scalar(0, 255, 0), 2);

    #if QT == true
    cv::imshow("contours", contour_vis);
    #endif
    
    double bestScore = 0.0;
    cv::Rect bestRect;
    std::vector<cv::Point> bestApprox;
    double maxAspect = 0.;
    for (const auto &c : contours) {
      double area = cv::contourArea(c);
      //RCLCPP_INFO(this->get_logger(), "aspect from detect %.3f", aspect);
      if (area < 400.0){

        continue; // фильтр по площади (подстрой)
      }
      cv::Rect r = cv::boundingRect(c);
      double aspect = (double)r.width / (double)r.height;
      if (aspect > maxAspect) {
        maxAspect = aspect;
      }
      std::cout<< "aspect from detect"<<aspect<<" area:"<< area<<std::endl;
      if (aspect < 0.15 || aspect > 3.0){

        continue; // близко к квадрату
      }
      // Аппроксимация контура -> квадрат обычно даёт 4 вершины
      //*
      std::vector<cv::Point> approx;
      double peri = cv::arcLength(c, true);
      cv::approxPolyDP(c, approx, 0.02 * peri, true);
      if ((int)approx.size() >= 15){
        std::cout<< "больше 15 вершин"<<std::endl;
        continue;
      }
      /*
      if (!cv::isContourConvex(approx)){ // выпуклость
        std::cout<< "не выпуклый контур"<<std::endl;
        continue;
      }*/
      //*/
      
      double fill = area / (double)(r.area() + 1);
      double score = area * fill;

      if (score > bestScore) {
        bestScore = score;
        bestRect = r;
        bestApprox = approx;
      }
    }
    
    if (bestScore <= 0.0){
      std::cout<< "bestScore <=0"<<std::endl;
      return d;
    }

    d.found = true;
    d.bbox = bestRect;
    d.center = cv::Point2f(bestRect.x + bestRect.width * 0.5f,
                           bestRect.y + bestRect.height * 0.5f);

    // Определяем “клетку” сетки grid_cols x grid_rows
    const int W = bgr.cols;
    const int H = bgr.rows;

    int cellW = std::max(1, W / grid_cols);
    int cellH = std::max(1, H / grid_rows);
    
    d.cell_x = std::clamp((int)(d.center.x / cellW), 0, grid_cols - 1);
    d.cell_y = std::clamp((int)(d.center.y / cellH), 0, grid_rows - 1);
    std::cout<< "detected "<<d.cell_x<<" "<<d.cell_y<<std::endl;
    return d;
  }

  std::vector<CloudPoint>
  onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::vector<CloudPoint> points;
    const auto cloud_t = rclcpp::Time(msg->header.stamp);
    const double dt_cloud = std::abs((cloud_t - last_image_stamp_).seconds());
    const double det_age = std::abs((cloud_t - last_detection_stamp_).seconds());
    const bool detection_fresh = last_detection.found && det_age <= detection_hold_sec_;

    if (dt_cloud > 3.0 || !detection_fresh) {
      TargetState tmp;
      MotionCommand cmd = compute(tmp, FollowMode::LOST);
#if CONTROL == true
      sendCommand(cmd);
#endif
      return points;
    }

    const int src_w = std::max(1, image_width_);
    const int src_h = std::max(1, image_height_);
    const int cloud_w = static_cast<int>(msg->width);
    const int cloud_h = static_cast<int>(msg->height);
    const bool organized = cloud_w > 0 && cloud_h > 1;

    float fx = CAMERA_WIDTH / (2.0f * std::tan(CAMERA_FOV / 2.0f));
    float cx = CAMERA_WIDTH / 2.0f;
    float u_left = static_cast<float>(last_detection.bbox.x);
    float u_right = static_cast<float>(last_detection.bbox.x + last_detection.bbox.width);
    float theta_left = std::atan2(u_left - cx, fx);
    float theta_right = std::atan2(u_right - cx, fx);
    if (theta_left > theta_right) {
      std::swap(theta_left, theta_right);
    }

    cv::Rect roi;
    if (organized) {
      const float sx = static_cast<float>(cloud_w) / static_cast<float>(src_w);
      const float sy = static_cast<float>(cloud_h) / static_cast<float>(src_h);
      roi.x = std::clamp(static_cast<int>(last_detection.bbox.x * sx), 0, cloud_w - 1);
      roi.y = std::clamp(static_cast<int>(last_detection.bbox.y * sy), 0, cloud_h - 1);
      roi.width = std::clamp(static_cast<int>(last_detection.bbox.width * sx), 1, cloud_w - roi.x);
      roi.height = std::clamp(static_cast<int>(last_detection.bbox.height * sy), 1, cloud_h - roi.y);
    }

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      size_t idx = 0;
      points.reserve(5000);

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++idx) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
          continue;
        const float range = std::sqrt(x * x + y * y + z * z);
        if (!std::isfinite(range) || range <= 0.05f || range > 20.0f)
          continue;

        if (organized) {
          const int u = static_cast<int>(idx % static_cast<size_t>(cloud_w));
          const int v = static_cast<int>(idx / static_cast<size_t>(cloud_w));
          if (u < roi.x || u >= roi.x + roi.width || v < roi.y || v >= roi.y + roi.height)
            continue;
        } else {
          const float angle = std::atan2(x, z);
          if (angle < theta_left || angle > theta_right)
            continue;
        }

        CloudPoint p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.range = range;
        p.angle = std::atan2(x, z);
        points.push_back(p);
      }
    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR(this->get_logger(), "PointCloud2 parse error: %s", e.what());
      return points;
    }

    processTogether(points, last_detection);
    return points;
  }

  std::string win_ = "PointCloud ROI";
  double maxAspect = 0.;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<LidarViewer>());
  auto node = std::make_shared<ImageViewer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
