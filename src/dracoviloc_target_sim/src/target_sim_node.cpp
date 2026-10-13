#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace fs = std::filesystem;

// =============================================================================
// target_sim_node  -  synthetic moving bearing for validating arm_tracking
//
// Impersonates the real YOLO detector at the message level: it publishes the
// same /yolo/direction (geometry_msgs/Vector3Stamped, unit vector in the
// tracking frame) and /yolo/target_marker (RViz arrow) that the real Isaac
// ROS yolo_camera_direction node publishes. dracoviloc_ekf and arm_tracker
// run completely unmodified -- they cannot tell this apart from a real
// detector. No Gazebo, no physics: a bearing has no range, so there is
// nothing physical to simulate, only a direction to move smoothly over time.
// =============================================================================

class TargetSim : public rclcpp::Node {
public:
  TargetSim() : Node("target_sim_node"), rng_(0) {
    tracking_frame_ = declare_parameter("tracking_frame", std::string("table_mic_link"));
    path_type_ = declare_parameter("path_type", std::string("sweep"));
    center_azimuth_deg_ = declare_parameter("center_azimuth_deg", 0.0);
    center_elevation_deg_ = declare_parameter("center_elevation_deg", 10.0);
    azimuth_amplitude_deg_ = declare_parameter("azimuth_amplitude_deg", 50.0);
    elevation_amplitude_deg_ = declare_parameter("elevation_amplitude_deg", 0.0);
    period_s_ = declare_parameter("period_s", 6.0);
    step_interval_s_ = declare_parameter("step_interval_s", 3.0);
    random_walk_step_deg_ = declare_parameter("random_walk_step_deg", 5.0);
    noise_stddev_deg_ = declare_parameter("noise_stddev_deg", 0.0);
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 30.0);
    marker_arrow_length_ = declare_parameter("marker_arrow_length", 1.0);
    seed_ = declare_parameter("seed", 42);
    target_log_path_ = declare_parameter("target_log_path", std::string(""));

    if (path_type_ != "sweep" && path_type_ != "circle" &&
        path_type_ != "step" && path_type_ != "random_walk") {
      throw std::invalid_argument(
        "path_type must be sweep, circle, step, or random_walk");
    }
    if (period_s_ <= 0.0 || step_interval_s_ <= 0.0) {
      throw std::invalid_argument("period_s and step_interval_s must be positive");
    }
    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }
    if (noise_stddev_deg_ < 0.0 || random_walk_step_deg_ < 0.0) {
      throw std::invalid_argument("noise_stddev_deg and random_walk_step_deg must be non-negative");
    }

    rng_.seed(static_cast<uint32_t>(seed_));
    walk_azimuth_deg_ = center_azimuth_deg_;
    walk_elevation_deg_ = center_elevation_deg_;
    step_azimuth_deg_ = center_azimuth_deg_;
    step_elevation_deg_ = center_elevation_deg_;

    direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/yolo/direction", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/yolo/target_marker", 10);

    open_target_log();

    start_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {tick();});

    RCLCPP_INFO(get_logger(),
      "Publishing synthetic %s bearings at %.1f Hz on /yolo/direction "
      "(tracking_frame=%s, center=(%.1f, %.1f) deg)",
      path_type_.c_str(), publish_rate_hz_, tracking_frame_.c_str(),
      center_azimuth_deg_, center_elevation_deg_);
  }

private:
  void open_target_log() {
    if (target_log_path_.empty()) {return;}
    const fs::path path(target_log_path_);
    if (!path.parent_path().empty()) {
      fs::create_directories(path.parent_path());
    }
    const bool write_header = !fs::exists(path) || fs::file_size(path) == 0;
    target_log_.open(path, std::ios::app);
    if (!target_log_) {
      throw std::runtime_error("cannot open target log: " + path.string());
    }
    if (write_header) {
      target_log_
        << "time_sec,path_type,true_azimuth_deg,true_elevation_deg,"
        << "published_azimuth_deg,published_elevation_deg\n";
    }
    target_log_ << std::setprecision(17);
    target_log_.flush();
    RCLCPP_INFO(get_logger(), "Logging ground-truth target path to %s", path.c_str());
  }

  // Degrees, centered on (center_azimuth_deg_, center_elevation_deg_).
  std::pair<double, double> true_bearing_deg(double t) {
    if (path_type_ == "sweep") {
      const double phase = 2.0 * M_PI * t / period_s_;
      return {
        center_azimuth_deg_ + azimuth_amplitude_deg_ * std::sin(phase),
        center_elevation_deg_ + elevation_amplitude_deg_ * std::sin(phase)};
    }
    if (path_type_ == "circle") {
      const double phase = 2.0 * M_PI * t / period_s_;
      return {
        center_azimuth_deg_ + azimuth_amplitude_deg_ * std::cos(phase),
        center_elevation_deg_ + elevation_amplitude_deg_ * std::sin(phase)};
    }
    if (path_type_ == "step") {
      if (t - last_step_time_ >= step_interval_s_) {
        last_step_time_ = t;
        std::uniform_real_distribution<double> az_dist(
          center_azimuth_deg_ - azimuth_amplitude_deg_,
          center_azimuth_deg_ + azimuth_amplitude_deg_);
        std::uniform_real_distribution<double> el_dist(
          center_elevation_deg_ - elevation_amplitude_deg_,
          center_elevation_deg_ + elevation_amplitude_deg_);
        step_azimuth_deg_ = az_dist(rng_);
        step_elevation_deg_ = elevation_amplitude_deg_ > 0.0 ? el_dist(rng_) : center_elevation_deg_;
      }
      return {step_azimuth_deg_, step_elevation_deg_};
    }
    // random_walk
    std::uniform_real_distribution<double> step_dist(
      -random_walk_step_deg_, random_walk_step_deg_);
    walk_azimuth_deg_ = std::clamp(
      walk_azimuth_deg_ + step_dist(rng_),
      center_azimuth_deg_ - azimuth_amplitude_deg_,
      center_azimuth_deg_ + azimuth_amplitude_deg_);
    if (elevation_amplitude_deg_ > 0.0) {
      walk_elevation_deg_ = std::clamp(
        walk_elevation_deg_ + step_dist(rng_),
        center_elevation_deg_ - elevation_amplitude_deg_,
        center_elevation_deg_ + elevation_amplitude_deg_);
    }
    return {walk_azimuth_deg_, walk_elevation_deg_};
  }

  void tick() {
    const auto stamp = now();
    const double t = (stamp - start_time_).seconds();
    const auto [true_az_deg, true_el_deg] = true_bearing_deg(t);

    double published_az_deg = true_az_deg;
    double published_el_deg = true_el_deg;
    if (noise_stddev_deg_ > 0.0) {
      std::normal_distribution<double> noise(0.0, noise_stddev_deg_);
      published_az_deg += noise(rng_);
      published_el_deg += noise(rng_);
    }

    const double az = published_az_deg * M_PI / 180.0;
    const double el = published_el_deg * M_PI / 180.0;
    const double ce = std::cos(el);

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.stamp = stamp;
    direction.header.frame_id = tracking_frame_;
    direction.vector.x = ce * std::cos(az);
    direction.vector.y = ce * std::sin(az);
    direction.vector.z = std::sin(el);
    direction_pub_->publish(direction);

    visualization_msgs::msg::Marker marker;
    marker.header = direction.header;
    marker.ns = "target_sim";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.points.emplace_back();
    geometry_msgs::msg::Point endpoint;
    endpoint.x = marker_arrow_length_ * direction.vector.x;
    endpoint.y = marker_arrow_length_ * direction.vector.y;
    endpoint.z = marker_arrow_length_ * direction.vector.z;
    marker.points.push_back(endpoint);
    marker.scale.x = 0.025;
    marker.scale.y = 0.06;
    marker.scale.z = 0.09;
    marker.color.r = 0.10;
    marker.color.g = 0.40;
    marker.color.b = 0.95;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(2.0 / publish_rate_hz_);
    marker_pub_->publish(marker);

    if (target_log_.is_open()) {
      target_log_
        << t << ',' << path_type_ << ',' << true_az_deg << ',' << true_el_deg << ','
        << published_az_deg << ',' << published_el_deg << '\n';
      target_log_.flush();
    }
  }

  std::string tracking_frame_, path_type_, target_log_path_;
  double center_azimuth_deg_, center_elevation_deg_;
  double azimuth_amplitude_deg_, elevation_amplitude_deg_;
  double period_s_, step_interval_s_, random_walk_step_deg_;
  double noise_stddev_deg_, publish_rate_hz_, marker_arrow_length_;
  int seed_;
  double last_step_time_{-1.0e9};
  double step_azimuth_deg_{0.0}, step_elevation_deg_{0.0};
  double walk_azimuth_deg_{0.0}, walk_elevation_deg_{0.0};
  std::mt19937 rng_;
  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::ofstream target_log_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetSim>());
  rclcpp::shutdown();
  return 0;
}
