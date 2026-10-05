#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class DirectionMarkerVisualizer : public rclcpp::Node
{
public:
  DirectionMarkerVisualizer()
  : Node("direction_marker_visualizer")
  {
    arrow_length_ = declare_parameter("arrow_length", 1.0);
    timeout_ = declare_parameter("marker_timeout", 1.25);
    if (!std::isfinite(arrow_length_) || arrow_length_ <= 0.0) {
      throw std::invalid_argument("arrow_length must be finite and positive");
    }
    if (!std::isfinite(timeout_) || timeout_ <= 0.0) {
      throw std::invalid_argument("marker_timeout must be finite and positive");
    }

    add_source(
      "mobilenetv2_enabled", "/mobilenetv2/direction",
      "/mobilenetv2/target_marker", "mobilenetv2_target", 0.95, 0.10, 0.85);
    add_source(
      "gre_enabled", "/gre/direction",
      "/gre/target_marker", "gre_target", 0.10, 0.95, 0.95);
    add_source(
      "ekf_enabled", "/ekf/direction",
      "/ekf/target_marker", "ekf_target", 1.00, 0.70, 0.05);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {expire_stale_markers();});
  }

private:
  using Direction = geometry_msgs::msg::Vector3Stamped;
  using Marker = visualization_msgs::msg::Marker;

  struct Source
  {
    std::string marker_namespace;
    float red;
    float green;
    float blue;
    std::string last_frame;
    rclcpp::Time last_received{0, 0, RCL_ROS_TIME};
    bool visible{false};
    rclcpp::Subscription<Direction>::SharedPtr subscription;
    rclcpp::Publisher<Marker>::SharedPtr publisher;
  };

  void add_source(
    const std::string & enabled_parameter, const std::string & direction_topic,
    const std::string & marker_topic, const std::string & marker_namespace,
    float red, float green, float blue)
  {
    if (!declare_parameter(enabled_parameter, false)) {
      return;
    }

    auto source = std::make_shared<Source>();
    source->marker_namespace = marker_namespace;
    source->red = red;
    source->green = green;
    source->blue = blue;
    source->publisher = create_publisher<Marker>(marker_topic, rclcpp::QoS(10).reliable());
    source->subscription = create_subscription<Direction>(
      direction_topic, rclcpp::QoS(10).reliable(),
      [this, source](const Direction::SharedPtr message) {
        publish_marker(*source, *message);
      });
    sources_.push_back(source);
    RCLCPP_INFO(
      get_logger(), "Visualizing %s on %s", direction_topic.c_str(), marker_topic.c_str());
  }

  void publish_marker(Source & source, const Direction & direction)
  {
    const double x = direction.vector.x;
    const double y = direction.vector.y;
    const double z = direction.vector.z;
    const double norm = std::sqrt(x * x + y * y + z * z);
    if (direction.header.frame_id.empty() || !std::isfinite(norm) || norm < 1.0e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Ignoring invalid direction for marker namespace %s",
        source.marker_namespace.c_str());
      return;
    }

    Marker marker;
    marker.header = direction.header;
    marker.ns = source.marker_namespace;
    marker.id = 0;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.points.emplace_back();
    geometry_msgs::msg::Point endpoint;
    endpoint.x = arrow_length_ * x / norm;
    endpoint.y = arrow_length_ * y / norm;
    endpoint.z = arrow_length_ * z / norm;
    marker.points.push_back(endpoint);
    marker.scale.x = 0.025;
    marker.scale.y = 0.06;
    marker.scale.z = 0.09;
    marker.color.r = source.red;
    marker.color.g = source.green;
    marker.color.b = source.blue;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(timeout_);
    source.publisher->publish(marker);

    source.last_frame = direction.header.frame_id;
    source.last_received = now();
    source.visible = true;
  }

  void expire_stale_markers()
  {
    const auto current_time = now();
    for (const auto & source : sources_) {
      if (!source->visible || (current_time - source->last_received).seconds() <= timeout_) {
        continue;
      }
      Marker marker;
      marker.header.stamp = current_time;
      marker.header.frame_id = source->last_frame;
      marker.ns = source->marker_namespace;
      marker.id = 0;
      marker.action = Marker::DELETE;
      source->publisher->publish(marker);
      source->visible = false;
    }
  }

  double arrow_length_;
  double timeout_;
  std::vector<std::shared_ptr<Source>> sources_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionMarkerVisualizer>());
  rclcpp::shutdown();
  return 0;
}
