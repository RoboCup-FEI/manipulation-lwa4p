#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "grasp_point_cv_interfaces/srv/get_grasp_candidates.hpp"

using Pose = geometry_msgs::msg::Pose;
using GetGraspCandidates = grasp_point_cv_interfaces::srv::GetGraspCandidates;

std::mutex image_mutex;
cv::Mat latest_image_rgb;
cv::Mat latest_image_depth;
builtin_interfaces::msg::Time latest_rgb_stamp;  // capture time of the color frame
builtin_interfaces::msg::Time latest_depth_stamp;

double fixed_z = 0.0;  // > 0 -> TEMPORARY stub for a bag without depth (Z = fixed_z)

double fx = 0.0;
double fy = 0.0;
double cx = 0.0;
double cy = 0.0;  // camera intrinsic parameters

rclcpp::Logger logger = rclcpp::get_logger("grasp_point_cv");
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub;

// Forward declarations (createPose was used before it was declared -> compile error)
Pose createPose(double x, double y, double z, double roll, double pitch, double yaw);
double Median(std::vector<std::uint16_t> depth_m, int n);

void imageCallback_depth(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(image_mutex);
  latest_image_depth = cv_bridge::toCvCopy(msg, "16UC1")->image;
  latest_depth_stamp = msg->header.stamp;
}

void imageCallback_rgb(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(image_mutex);
  latest_image_rgb = cv_bridge::toCvCopy(msg, "bgr8")->image;
  latest_rgb_stamp = msg->header.stamp;
}

void imageCallback_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(image_mutex);
  fx = msg->k[0];
  cx = msg->k[2];
  fy = msg->k[4];
  cy = msg->k[5];
}

void handleGetGraspCandidates(
  const std::shared_ptr<GetGraspCandidates::Request>/*request*/,
  std::shared_ptr<GetGraspCandidates::Response> response)
{
  cv::Mat image;
  cv::Mat depth_image;
  builtin_interfaces::msg::Time rgb_stamp, depth_stamp;
  double fx_copy, fy_copy, cx_copy, cy_copy;

  {
    std::lock_guard<std::mutex> lock(image_mutex);
    image = latest_image_rgb.clone();
    depth_image = latest_image_depth.clone();
    rgb_stamp = latest_rgb_stamp;
    depth_stamp = latest_depth_stamp;

    fx_copy = fx;
    fy_copy = fy;
    cx_copy = cx;
    cy_copy = cy;
  }

  response->success = false;
  response->poses.header.frame_id = "camera_color_optical_frame";

  if (image.empty()) {
    response->message = "no color frame received yet";
    return;
  }
  if (fx_copy <= 0.0 || fy_copy <= 0.0) {
    response->message = "no camera_info received yet";
    return;
  }

  const bool use_depth = (fixed_z <= 0.0);
  if (use_depth) {
    if (depth_image.empty()) {
      response->message = "no aligned depth frame received yet";
      return;
    }
    // color and depth must be from (almost) the same moment
    double dt = std::abs((rclcpp::Time(rgb_stamp) - rclcpp::Time(depth_stamp)).seconds());
    if (dt > 0.1) {
      response->message = "color and depth not synchronized, dt = " + std::to_string(dt) + " s";
      return;
    }
  }

  // stamp = capture time of the frame, NOT now()
  response->poses.header.stamp = rgb_stamp;

  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 16, 100, 30, 25, 60);

  if (circles.empty()) {
    std_msgs::msg::Header header;
    header.stamp = rgb_stamp;
    header.frame_id = "camera_color_optical_frame";
    debug_pub->publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());

    response->message = "no circles found";
    return;
  }

  // (score, pose)
  std::vector<std::pair<float, Pose>> candidates;

  // Process each detected circle
  for (size_t i = 0; i < circles.size(); i++) {
    cv::Vec3f c = circles[i];
    cv::Point center(cvRound(c[0]), cvRound(c[1]));
    // circle center coordinates
    int u = center.x;
    int v = center.y;

    double depth_m = 0.0;
    int valid_count = 25;

    if (use_depth) {
      // Collect depth values from a 5x5 neighborhood around the detected circle center
      std::vector<std::uint16_t> depth_values;
      for (int row = 0; row < 5; row++) {
        for (int col = 0; col < 5; col++) {
          int x = u + col - 2;
          int y = v + row - 2;

          if (x >= 0 && x < depth_image.cols && y >= 0 && y < depth_image.rows) {
            std::uint16_t depth_value = depth_image.at<std::uint16_t>(y, x);

            if (depth_value != 0) {
              depth_values.push_back(depth_value);
            }
          }
        }
      }
      valid_count = static_cast<int>(depth_values.size());

      // Calculate median depth
      if (!depth_values.empty()) {
        double median_mm = Median(depth_values, depth_values.size());
        depth_m = median_mm / 1000.0;
      }
    } else {
      depth_m = fixed_z;  // TEMPORARY stub
    }

    if (depth_m <= 0.0) {
      continue;
    }

    // pinhole camera
    double X = ((u - cx_copy) * depth_m) / fx_copy;
    double Y = ((v - cy_copy) * depth_m) / fy_copy;
    double Z = depth_m;

    // score: Hough order (first = stronger) x fraction of valid depth pixels
    float score = (1.0f / (1.0f + i)) * (valid_count / 25.0f);

    // circle -> theta = 0 -> identity quaternion
    candidates.push_back({score, createPose(X, Y, Z, 0.0, 0.0, 0.0)});

    RCLCPP_INFO(
      logger, "circle %zu: u=%d v=%d r=%.0f | X=%.3f Y=%.3f Z=%.3f m | depth px %d/25 | score %.3f",
      i, u, v, c[2], X, Y, Z, valid_count, score);

    // debug drawing
    int radius = cvRound(c[2]);
    cv::circle(image, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
    cv::circle(image, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    cv::putText(
      image, "Z=" + std::to_string(Z).substr(0, 5), center + cv::Point(8, -8),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
  }

  std_msgs::msg::Header header;
  header.stamp = rgb_stamp;
  header.frame_id = "camera_color_optical_frame";
  debug_pub->publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());

  if (candidates.empty()) {
    response->message = std::to_string(circles.size()) + " circles found, but none has valid depth";
    return;
  }

  // best first
  std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.first > b.first;
  });

  for (const auto & cand : candidates) {
    response->scores.push_back(cand.first);
    response->poses.poses.push_back(cand.second);
  }

  response->success = true;
  response->message = "found " + std::to_string(candidates.size()) + " candidates" +
    (use_depth ? "" : " [FIXED Z STUB]");
}

Pose createPose(double x, double y, double z, double roll, double pitch, double yaw)
{
  Pose pose;

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose;
}

double Median(std::vector<std::uint16_t> depth_m, int n)
{
  std::sort(depth_m.begin(), depth_m.end());

  n = depth_m.size();

  if (n == 0) {
    return 0.0;
  }

  if (n % 2 != 0) {
    return static_cast<double>(depth_m[n / 2]);
  }

  return (static_cast<double>(depth_m[(n - 1) / 2]) + depth_m[n / 2]) / 2.0;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grasp_point_cv");
  logger = node->get_logger();

  fixed_z = node->declare_parameter<double>("fixed_z", 0.0);

  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/color/image_raw", 10, imageCallback_rgb);

  auto image_sub2 = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/camera/aligned_depth_to_color/image_raw", 10, imageCallback_depth);

  auto image_sub3 = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera/color/camera_info", 10, imageCallback_info);

  debug_pub = node->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 1);

  auto service =
    node->create_service<GetGraspCandidates>("~/get_grasp_candidates", &handleGetGraspCandidates);

  RCLCPP_INFO(logger, "grasp_point_cv ready");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
