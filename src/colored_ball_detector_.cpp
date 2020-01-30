/*
 * COPYRIGHT Pedro Paulo Ventura Tecchio 2020
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"

const std::string OPENCV_WINDOW_ORIGINAL = "Original";
const std::string OPENCV_WINDOW_TRANSFORMED = "Transformed";

const int LOW_H  = 50, LOW_S  = 0.20*255, LOW_V  = 0.50*255;
const int HIGH_H = 62, HIGH_S = 1.00*255, HIGH_V = 1.00*255;

class ColoredBallDetector {
  cv::Mat image_src_;
  cv::Mat image_rgb_[3];

 public:
  ColoredBallDetector();
  ~ColoredBallDetector();
  void imageCb(const sensor_msgs::ImageConstPtr& _msg);

};

ColoredBallDetector::ColoredBallDetector() {
  cv::namedWindow(OPENCV_WINDOW_ORIGINAL, cv::WINDOW_NORMAL);
  // cv::namedWindow(OPENCV_WINDOW_TRANSFORMED, cv::WINDOW_NORMAL);
}

ColoredBallDetector::~ColoredBallDetector() {
  cv::destroyWindow(OPENCV_WINDOW_ORIGINAL);
  // cv::destroyWindow(OPENCV_WINDOW_TRANSFORMED);
}

void ColoredBallDetector::imageCb(const sensor_msgs::ImageConstPtr& _msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  ROS_INFO_STREAM("width: " << cv_ptr->image.rows <<
                  "height: " << cv_ptr->image.cols);

  // Store image received
  image_src_ = cv_ptr->image;
  // Convert image to HSV
  cv::Mat image_hsv;
  cv::cvtColor(image_src_, image_hsv, cv::COLOR_BGR2HSV);
  // Color segmentation
  cv::Mat image_threshold;
  cv::inRange(image_hsv,
              cv::Scalar(LOW_H, LOW_S, LOW_V),
              cv::Scalar(HIGH_H, HIGH_S, HIGH_V),
              image_threshold);

  // Show images
  cv::imshow(OPENCV_WINDOW_ORIGINAL, image_src_);
  cv::imshow(OPENCV_WINDOW_TRANSFORMED, image_threshold);




  // // from https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
  // cv::Mat gray;
  // cv::cvtColor(image_src_, gray, cv::COLOR_BGR2GRAY);
  // cv::medianBlur(gray, gray, 5);
  // std::vector<cv::Vec3f> circles;
  // cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
  //              gray.rows/16, 100, 60, 1, 300);
  // for (size_t i = 0; i < circles.size(); i++) {
  //   cv::Vec3i c = circles[i];
  //   cv::Point center = cv::Point(c[0], c[1]);
  //   // circle center
  //   cv::circle(image_src_, center, 1, cv::Scalar(0, 100, 100),
  //               3, cv::LINE_AA);
  //   // circle outline
  //   int radius = c[2];
  //   cv::circle(image_src_, center, radius, cv::Scalar(255, 0, 255),
  //               3, cv::LINE_AA);
  //   ROS_INFO("Circle %d at (%d, %d)", i, c[0], c[1]);
  // }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "colored_ball_detector");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  // image_transport::Publisher  image_pub;

  ColoredBallDetector cbd;

  image_sub = it.subscribe("/camera/image_raw", 1,
    &ColoredBallDetector::imageCb, &cbd);

  ros::spin();
  return 0;
}