/*
 * COPYRIGHT Pedro Paulo Ventura Tecchio 2020
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW_O = "Original";
static const std::string OPENCV_WINDOW_M = "Modified";
// const int LOW_H  = 20, LOW_S  = 0.00*255, LOW_V  = 0.00*255;
// const int HIGH_H = 60, HIGH_S = 1.00*255, HIGH_V = 1.00*255;

class ColoredBallDetector {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  // image_transport::Publisher image_pub_;
  const int max_value_H_ = 360/2;
  const int max_value_ = 255;
  int low_H_ = 20,
      low_S_ = 60,
      low_V_ = 60;
  int high_H_ = 30,
      high_S_ = 255,
      high_V_ = 255;
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
};

ColoredBallDetector::ColoredBallDetector() : it_(nh_) {
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("/camera/image_raw", 1,
    &ColoredBallDetector::imageCb, this);
  // image_pub_ = it_.advertise("/image_converter/output_video", 1);

  cv::namedWindow(OPENCV_WINDOW_O);
  cv::namedWindow(OPENCV_WINDOW_M);

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

ColoredBallDetector::~ColoredBallDetector() {
  cv::destroyWindow(OPENCV_WINDOW_O);
  cv::destroyWindow(OPENCV_WINDOW_M);
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

  cv::Mat image_segmented_color_not;
  cv::bitwise_not(image_segmented_color, image_segmented_color_not);

  // cv::GaussianBlur(image_segmented_color,
  //                  image_segmented_color,
  //                  cv::Size(13, 13),
  //                  0,
  //                  0,
  //                  cv::BORDER_REPLICATE);

  cv::Mat image_gray;
  cv::cvtColor(cv_ptr->image, image_gray, cv::COLOR_BGR2GRAY);
  cv::medianBlur(image_gray, image_gray, 5);

  cv::bitwise_xor(image_gray, image_gray,
                  image_gray, image_segmented_color_not);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(image_gray, circles, cv::HOUGH_GRADIENT, 1,
                   100, 100, 30, 20, 720);
  for (size_t i = 0; i < circles.size(); i++) {
    cv::Vec3i c = circles[i];
    cv::Point center = cv::Point(c[0], c[1]);
    // circle center
    cv::circle(cv_ptr->image, center, 1, cv::Scalar(0, 100, 100),
                3, cv::LINE_AA);
    // circle outline
    int radius = c[2];
    cv::circle(cv_ptr->image, center, radius, cv::Scalar(255, 0, 255),
                3, cv::LINE_AA);
    ROS_INFO("Circle %d at (%d, %d)", i, c[0], c[1]);
  }

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW_O, cv_ptr->image);
  cv::imshow(OPENCV_WINDOW_M, image_gray);
  cv::waitKey(3);

  // Output modified video stream
  // image_pub_.publish(cv_ptr->toImageMsg());
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

int main(int argc, char** argv){
  ros::init(argc, argv, "image_converter");
  ColoredBallDetector ic;
  ros::spin();
  return 0;
}