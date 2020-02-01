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
  void graphicalColorPickin(void);
};

ColoredBallDetector::ColoredBallDetector() : it_(nh_) {
  opened_windows = false;
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("/camera/image_raw", 1,
    &ColoredBallDetector::imageCb, this);
  // image_pub_ = it_.advertise("/image_converter/output_video", 1);
}

ColoredBallDetector::~ColoredBallDetector() {
  if (opened_windows) {
    cv::destroyWindow(OPENCV_WINDOW_O);
    cv::destroyWindow(OPENCV_WINDOW_M);
  }
}

void ColoredBallDetector::graphicalColorPickin(void) {
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
  std::vector<cv::Point2f> circle_centers(contours.size());
  std::vector<float> circle_radius(contours.size());

  for (int i = 0; i < contours.size(); i++) {
    cv::approxPolyDP(contours[i], contours_polygon[i],
      cv::arcLength(contours[i], true)*0.001, true);
    ROS_INFO_STREAM("idx: " << i << " # vertices: "
      << contours_polygon[i].size());



    cv::drawContours(cv_ptr->image, contours_polygon, i,
      cv::Scalar(0, 255, 0), cv::LINE_4, 1, hierarchy);
    cv::minEnclosingCircle(contours[i], circle_centers[i], circle_radius[i]);
    // cv::circle(cv_ptr->image, circle_centers[i], circle_radius[i],
    //   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);

    // cv::drawContours(cv_ptr->image, contours, i,
    //   cv::Scalar(255, 0, 0), cv::LINE_4, 1, hierarchy);

    cv::putText(cv_ptr->image, std::to_string(contours_polygon[i].size()),
              circle_centers[i], CV_FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 0, 0), 1, 8, false);

  }



  // std::vector<cv::Scalar> color = {cv::Scalar(255, 0, 0),
  //                                  cv::Scalar(0, 255, 0)};

  // std::vector<cv::RotatedRect> ellipse_boxes;
  // std::vector<cv::Moments> mu(contours.size());
  // std::vector<cv::Point2f> mc(contours.size());

  // cv::Mat ellipse_image = cv::Mat::zeros(image_segmented_color.size(), CV_8UC1);

  // for (int i = 0; i < contours.size(); i++) {
  //   if (contours[i].size() < 6)
  //     continue;
  //   ellipse_boxes.push_back(cv::fitEllipse(contours[i]));
  //   cv::ellipse(ellipse_image, ellipse_boxes[i],
  //     cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
  // }

  // std::vector<std::vector<cv::Point>> contours_ellipse;
  // std::vector<cv::Vec4i> hierarchy_ellipse;
  // cv::findContours(ellipse_image,
  //                  contours_ellipse,
  //                  hierarchy_ellipse,
  //                  cv::RETR_EXTERNAL,
  //                  cv::CHAIN_APPROX_SIMPLE,
  //                  cv::Point(0, 0));

  // std::vector<double> contour_mismatch;
  // for (int i = 0; i < contours.size(); i++) {
  //   contour_mismatch.push_back(cv::matchShapes(contours[i], contours_ellipse[i],
  //     CV_CONTOURS_MATCH_I2, 0));

  //   mu[i] = cv::moments(contours_ellipse[i], false);
  //   mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

  //   cv::putText(cv_ptr->image, std::to_string(contour_mismatch[i]),
  //               mc[i], CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0),
  //               1, 8, false);
  // }






  // Image processing for HoughCircles

  // cv::Mat image_segmented_color_not;
  // cv::bitwise_not(image_segmented_color, image_segmented_color_not);

  // cv::Mat image_gray;
  // cv::cvtColor(cv_ptr->image, image_gray, cv::COLOR_BGR2GRAY);
  // cv::medianBlur(image_gray, image_gray, 5);

  // cv::bitwise_xor(image_gray, image_gray,
  //                 image_gray, image_segmented_color_not);

  // std::vector<cv::Vec3f> circles;
  // cv::HoughCircles(image_gray, circles, cv::HOUGH_GRADIENT, 1,
  //                  50, 100, 20, 20, 720);
  // for (size_t i = 0; i < circles.size(); i++) {
  //   cv::Vec3i c = circles[i];
  //   cv::Point center = cv::Point(c[0], c[1]);
  //   // circle center
  //   cv::circle(cv_ptr->image, center, 1, cv::Scalar(0, 100, 100),
  //               3, cv::LINE_AA);
  //   // circle outline
  //   int radius = c[2];
  //   cv::circle(cv_ptr->image, center, radius, cv::Scalar(255, 0, 255),
  //               3, cv::LINE_AA);
  //   ROS_INFO("Circle %d at (%d, %d)", i, c[0], c[1]);
  // }

  // Update GUI Window
  if (opened_windows) {
    cv::imshow(OPENCV_WINDOW_O, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW_M, image_segmented_color);
    cv::waitKey(3);
  }

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");
  ColoredBallDetector cbd;
  cbd.graphicalColorPickin();
  ros::spin();
  return 0;
}