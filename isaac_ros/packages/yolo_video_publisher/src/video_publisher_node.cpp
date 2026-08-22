// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class VideoPublisherNode : public rclcpp::Node
{
public:
  VideoPublisherNode()
  : Node("video_publisher")
  {
    const auto video_path = declare_parameter<std::string>("video_path", "");
    loop_ = declare_parameter<bool>("loop", false);
    const auto requested_rate = declare_parameter<double>("publish_rate", 0.0);
    const auto horizontal_fov_deg = declare_parameter<double>("horizontal_fov_deg", 0.0);
    const auto image_topic = declare_parameter<std::string>("image_topic", "/image");
    const auto camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera_info");

    if (video_path.empty()) {
      throw std::invalid_argument("The video_path parameter is required");
    }

    capture_.open(video_path);
    if (!capture_.isOpened()) {
      throw std::runtime_error("Could not open video: " + video_path);
    }

    width_ = static_cast<uint32_t>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    height_ = static_cast<uint32_t>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double source_rate = capture_.get(cv::CAP_PROP_FPS);
    publish_rate_ = requested_rate > 0.0 ? requested_rate : source_rate;
    if (publish_rate_ <= 0.0) {
      publish_rate_ = 30.0;
    }
    focal_length_pixels_ = horizontal_fov_deg > 0.0 ?
      static_cast<double>(width_) /
      (2.0 * std::tan(horizontal_fov_deg * std::acos(-1.0) / 360.0)) :
      static_cast<double>(width_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic, qos);
    camera_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, qos);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VideoPublisherNode::publish_frame, this));

    RCLCPP_INFO(
      get_logger(), "Publishing %s at %ux%u, %.2f FPS", video_path.c_str(), width_, height_,
      publish_rate_);
  }

private:
  void publish_frame()
  {
    cv::Mat bgr_frame;
    if (!capture_.read(bgr_frame)) {
      if (!loop_) {
        RCLCPP_INFO(get_logger(), "End of video");
        timer_->cancel();
        return;
      }
      capture_.set(cv::CAP_PROP_POS_FRAMES, 0.0);
      if (!capture_.read(bgr_frame)) {
        RCLCPP_ERROR(get_logger(), "Could not restart video");
        timer_->cancel();
        return;
      }
    }

    cv::Mat rgb_frame;
    cv::cvtColor(bgr_frame, rgb_frame, cv::COLOR_BGR2RGB);
    if (!rgb_frame.isContinuous()) {
      rgb_frame = rgb_frame.clone();
    }

    const auto stamp = now();
    auto image = std::make_unique<sensor_msgs::msg::Image>();
    image->header.stamp = stamp;
    image->header.frame_id = "video_frame";
    image->height = height_;
    image->width = width_;
    image->encoding = "rgb8";
    image->is_bigendian = false;
    image->step = width_ * 3;
    const auto byte_count = static_cast<size_t>(image->step) * image->height;
    image->data.resize(byte_count);
    std::memcpy(image->data.data(), rgb_frame.data, byte_count);

    auto camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
    camera_info->header = image->header;
    camera_info->height = height_;
    camera_info->width = width_;
    camera_info->distortion_model = "plumb_bob";
    camera_info->d.assign(5, 0.0);
    camera_info->k = {
      focal_length_pixels_, 0.0, width_ / 2.0,
      0.0, focal_length_pixels_, height_ / 2.0,
      0.0, 0.0, 1.0};
    camera_info->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info->p = {
      focal_length_pixels_, 0.0, width_ / 2.0, 0.0,
      0.0, focal_length_pixels_, height_ / 2.0, 0.0,
      0.0, 0.0, 1.0, 0.0};

    camera_info_publisher_->publish(std::move(camera_info));
    image_publisher_->publish(std::move(image));
  }

  cv::VideoCapture capture_;
  uint32_t width_{0};
  uint32_t height_{0};
  double publish_rate_{30.0};
  double focal_length_pixels_{0.0};
  bool loop_{false};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VideoPublisherNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("video_publisher"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
