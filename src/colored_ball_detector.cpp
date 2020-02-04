/*
 * COPYRIGHT Pedro Paulo Ventura Tecchio 2020
*/

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW_O = "Original";
static const std::string OPENCV_WINDOW_M = "Modified";

class ColoredBallDetector {
  ros::NodeHandle nh_;
  ros::Publisher pubPointStamped_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  std::size_t id_threshold_ = 50;
  const double camera_fov = M_PI / 1.5;
  const int max_value_H_ = 360/2;
  const int max_value_ = 255;
  int low_H_ = 20,
      low_S_ = 60,
      low_V_ = 60;
  int high_H_ = 30,
      high_S_ = 255,
      high_V_ = 255;
  bool opened_windows = false;
  static void on_low_H_thresh_trackbar(int v , void *ptr) {
    // resolve 'this':
    ColoredBallDetector *that = reinterpret_cast<ColoredBallDetector*>(ptr);
    that->on_low_H_thresh_trackbar(v);
  }
  void on_low_H_thresh_trackbar(int v);
  static void on_high_H_thresh_trackbar(int v , void *ptr) {
    // resolve 'this':
    ColoredBallDetector *that = reinterpret_cast<ColoredBallDetector*>(ptr);
    that->on_high_H_thresh_trackbar(v);
  }
  void on_high_H_thresh_trackbar(int v);
  static void on_low_S_thresh_trackbar(int v , void *ptr) {
    // resolve 'this':
    ColoredBallDetector *that = reinterpret_cast<ColoredBallDetector*>(ptr);
    that->on_low_S_thresh_trackbar(v);
  }
  void on_low_S_thresh_trackbar(int v);
  static void on_high_S_thresh_trackbar(int v , void *ptr) {
    // resolve 'this':
    ColoredBallDetector *that = reinterpret_cast<ColoredBallDetector*>(ptr);
    that->on_high_S_thresh_trackbar(v);
  }
  void on_high_S_thresh_trackbar(int v);
  static void on_low_V_thresh_trackbar(int v , void *ptr) {
    // resolve 'this':
    ColoredBallDetector *that = reinterpret_cast<ColoredBallDetector*>(ptr);
    that->on_low_V_thresh_trackbar(v);
  }
  void on_low_V_thresh_trackbar(int v);
  static void on_high_V_thresh_trackbar(int v , void *ptr) {
    // resolve 'this':
    ColoredBallDetector *that = reinterpret_cast<ColoredBallDetector*>(ptr);
    that->on_high_V_thresh_trackbar(v);
  }
  void on_high_V_thresh_trackbar(int v);

 public:
  ColoredBallDetector();
  ~ColoredBallDetector();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void graphicalColorPicking(void);
};

ColoredBallDetector::ColoredBallDetector() : it_(nh_) {
  opened_windows = false;
  image_sub_ = it_.subscribe("/camera/image_raw", 1,
    &ColoredBallDetector::imageCb, this);
  pubPointStamped_ = nh_.advertise<geometry_msgs::PointStamped>
    ("colored_ball_detector/yaw", 1);
}

ColoredBallDetector::~ColoredBallDetector() {
  if (opened_windows) {
    cv::destroyWindow(OPENCV_WINDOW_O);
    cv::destroyWindow(OPENCV_WINDOW_M);
  }
}

void ColoredBallDetector::graphicalColorPicking(void) {
  if (!opened_windows) {
    cv::namedWindow(OPENCV_WINDOW_O);
    cv::namedWindow(OPENCV_WINDOW_M);
    opened_windows = true;
  }
  // Trackbars to set thresholds for HSV values
  cv::createTrackbar("Low H",
                     OPENCV_WINDOW_O,
                     &low_H_,
                     max_value_H_,
                     on_low_H_thresh_trackbar,
                     this);
  cv::createTrackbar("High H",
                     OPENCV_WINDOW_O,
                     &high_H_,
                     max_value_H_,
                     on_high_H_thresh_trackbar,
                     this);
  cv::createTrackbar("Low S",
                     OPENCV_WINDOW_O,
                     &low_S_,
                     max_value_,
                     on_low_S_thresh_trackbar,
                     this);
  cv::createTrackbar("High S",
                     OPENCV_WINDOW_O,
                     &high_S_,
                     max_value_,
                     on_high_S_thresh_trackbar,
                     this);
  cv::createTrackbar("Low V",
                     OPENCV_WINDOW_O,
                     &low_V_,
                     max_value_,
                     on_low_V_thresh_trackbar,
                     this);
  cv::createTrackbar("High V",
                     OPENCV_WINDOW_O,
                     &high_V_,
                     max_value_,
                     on_high_V_thresh_trackbar,
                     this);
}

void ColoredBallDetector::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert image to HSV
  cv::Mat image_hsv;
  cv::cvtColor(cv_ptr->image, image_hsv, cv::COLOR_BGR2HSV);
  // Color segmentation
  cv::Mat image_segmented_color;
  cv::inRange(image_hsv,
              cv::Scalar(low_H_, low_S_, low_V_),
              cv::Scalar(high_H_, high_S_, high_V_),
              image_segmented_color);
  cv::medianBlur(image_segmented_color, image_segmented_color, 5);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours(image_segmented_color,
                   contours,
                   hierarchy,
                   cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE,
                   cv::Point(0, 0));
  if (contours.size() == 0)
    return;

  std::vector<std::vector<cv::Point>> contours_polygon(contours.size());
  std::vector<cv::Moments> moments(contours.size());
  std::vector<cv::Point2f> centers(contours.size());
  std::size_t max = 0;
  int id = -1;

  for (int i = 0; i < contours.size(); i++) {
    cv::approxPolyDP(contours[i], contours_polygon[i],
      cv::arcLength(contours[i], true)*0.001, true);

    // computing polygon centers
    moments[i] = cv::moments(contours[i], false);
    centers[i] = cv::Point2f(moments[i].m10 / moments[i].m00,
      moments[i].m01 / moments[i].m00);

    cv::drawContours(cv_ptr->image, contours_polygon, i,
      cv::Scalar(0, 255, 0), cv::LINE_4, 1, hierarchy);

    cv::putText(cv_ptr->image, std::to_string(contours_polygon[i].size()),
              centers[i], CV_FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 0, 0), 1, 8, false);
    cv::circle(cv_ptr->image, centers[i], 4, cv::Scalar(0, 255, 0),
      2, cv::FILLED, 0);

    // Select candidate for circle from maximum number of vertices
    // from polygon approximation
    if (contours_polygon[i].size() > id_threshold_) {
      if (max < contours_polygon[i].size()) {
        max = contours_polygon[i].size();
        id = i;
      }
    }
  }

  if (id != -1) {

    // ROS_INFO_STREAM("idx: " << id);
    // ROS_INFO_STREAM("# vertices: " << max);
    // ROS_INFO_STREAM("Cx: " << centers[id].x);
    // ROS_INFO_STREAM("Cy: " << centers[id].y);


    // getting header info from camera msg
    std_msgs::Header outputHeaderMsg;
    outputHeaderMsg.seq = msg->header.seq;
    outputHeaderMsg.stamp = ros::Time::now();
    outputHeaderMsg.frame_id = msg->header.frame_id;

    geometry_msgs::PointStamped outputPointStampedMsg;
    outputPointStampedMsg.header = outputHeaderMsg;
    double dx = centers[id].x - msg->width / 2;
    double dy = centers[id].y - msg->height / 2;
    // double areaCirculo = M_PI * 0.5 * 0.5;
    double areaImagem = msg->width * msg->height;
    double area = contourArea(contours_polygon[id], false);
    outputPointStampedMsg.point.x =
      abs(area / areaImagem);
    outputPointStampedMsg.point.y =
      abs((areaImagem - area) / areaImagem);
    outputPointStampedMsg.point.z = -(dx * camera_fov / msg->width);

    pubPointStamped_.publish(outputPointStampedMsg);
  }


  // Update GUI Window
  if (opened_windows) {
    cv::imshow(OPENCV_WINDOW_O, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW_M, image_segmented_color);
    cv::waitKey(3);
  }
}

void ColoredBallDetector::on_low_H_thresh_trackbar(int v) {
    low_H_ = cv::min(high_H_ - 1, low_H_);
    cv::setTrackbarPos("Low H", OPENCV_WINDOW_O, low_H_);
}
void ColoredBallDetector::on_high_H_thresh_trackbar(int v) {
    high_H_ = cv::max(high_H_, low_H_ + 1);
    cv::setTrackbarPos("High H", OPENCV_WINDOW_O, high_H_);
}
void ColoredBallDetector::on_low_S_thresh_trackbar(int v) {
    low_S_ = cv::min(high_S_ - 1, low_S_);
    cv::setTrackbarPos("Low S", OPENCV_WINDOW_O, low_S_);
}
void ColoredBallDetector::on_high_S_thresh_trackbar(int v) {
    high_S_ = cv::max(high_S_, low_S_ + 1);
    cv::setTrackbarPos("High S", OPENCV_WINDOW_O, high_S_);
}
void ColoredBallDetector::on_low_V_thresh_trackbar(int v) {
    low_V_ = cv::min(high_V_ - 1, low_V_);
    cv::setTrackbarPos("Low V", OPENCV_WINDOW_O, low_V_);
}
void ColoredBallDetector::on_high_V_thresh_trackbar(int v) {
    high_V_ = cv::max(high_V_, low_V_+ 1);
    cv::setTrackbarPos("High V", OPENCV_WINDOW_O, high_V_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");
  ColoredBallDetector cbd;
  cbd.graphicalColorPicking();
  ros::spin();
  return 0;
}