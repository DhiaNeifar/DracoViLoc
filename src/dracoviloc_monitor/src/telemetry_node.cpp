// telemetry_node: one compact status snapshot of the whole pipeline.
//
// Publishes std_msgs/String (JSON) on /dracoviloc/telemetry at `publish_rate_hz`
// and, when `log_path` is set, appends snapshots to that JSONL file at
// `log_rate_hz`. Everything heavy (rates, derivatives, aggregation) happens
// here in C++, so the GUI only parses ~2 KB ten times per second.
//
// Snapshot sections: camera, yolo, mobilenetv2, ekf, arms (per-joint velocity,
// acceleration, jerk), ee (end-effector speed/acceleration/jerk), audio (ODAS
// tracked sources). Directions are unit vectors in their message frame, with
// azimuth = atan2(y, x) and elevation = asin(z) in degrees.
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <dracoviloc_msgs/msg/classifier_window.hpp>
#include <dracoviloc_msgs/msg/ekf_update.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <odas_ros_msgs/msg/odas_sst_array_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "dracoviloc_monitor/monitor_utils.hpp"

using dracoviloc_monitor::JointDerivatives;
using dracoviloc_monitor::JsonWriter;
using dracoviloc_monitor::RateMeter;

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + stamp.nanosec * 1e-9;
}
}  // namespace

class TelemetryNode : public rclcpp::Node
{
public:
  TelemetryNode()
  : Node("dracoviloc_telemetry")
  {
    const double publish_rate = declare_parameter<double>("publish_rate_hz", 10.0);
    const double log_rate = declare_parameter<double>("log_rate_hz", 2.0);
    const auto log_path = declare_parameter<std::string>("log_path", "");
    const double joint_alpha = declare_parameter<double>("joint_filter_alpha", 0.3);
    if (publish_rate <= 0.0 || log_rate <= 0.0 || joint_alpha <= 0.0 || joint_alpha > 1.0) {
      throw std::invalid_argument("invalid telemetry parameters");
    }
    joint_alpha_ = joint_alpha;
    if (!log_path.empty()) {
      std::filesystem::create_directories(std::filesystem::path(log_path).parent_path());
      log_.open(log_path, std::ios::app);
      if (!log_) {
        throw std::runtime_error("cannot open telemetry log " + log_path);
      }
      RCLCPP_INFO(get_logger(), "telemetry log: %s (%.1f Hz)", log_path.c_str(), log_rate);
    }

    const auto sensor = rclcpp::SensorDataQoS();
    const auto reliable = rclcpp::QoS(50);
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", sensor, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double now = wall();
        camera_rate_.tick(now);
        camera_width_ = msg->width;
        camera_height_ = msg->height;
        camera_latency_ = now - stamp_seconds(msg->header.stamp);
      });
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detections_output", reliable, [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double now = wall();
        detection_rate_.tick(now);
        detections_ = static_cast<int>(msg->detections.size());
        best_score_ = 0.0;
        for (const auto & detection : msg->detections) {
          if (!detection.results.empty()) {
            best_score_ = std::max(best_score_, detection.results[0].hypothesis.score);
          }
        }
        if (detections_ > 0) {
          positive_rate_.tick(now);
        }
        yolo_latency_ = now - stamp_seconds(msg->header.stamp);
      });
    for (const auto & name : {"yolo", "mobilenetv2", "ekf"}) {
      directions_[name] = {};
      direction_subs_.push_back(create_subscription<geometry_msgs::msg::Vector3Stamped>(
        std::string("/") + name + "/direction", reliable,
        [this, name = std::string(name)](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          auto & direction = directions_[name];
          direction.rate.tick(wall());
          direction.vector = {msg->vector.x, msg->vector.y, msg->vector.z};
          direction.frame = msg->header.frame_id;
        }));
    }
    ekf_sub_ = create_subscription<dracoviloc_msgs::msg::EkfUpdate>(
      "/ekf/diagnostics", reliable, [this](dracoviloc_msgs::msg::EkfUpdate::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto & source = ekf_sources_[msg->source];
        source.rate.tick(wall());
        source.decisions[msg->decision] += 1;
        source.last_decision = msg->decision;
        source.last_nis = msg->nis;
        source.last_innovation_deg = msg->innovation_deg;
        source.last_delay_s = msg->processing_delay_s;
        ekf_nis_gate_ = msg->nis_gate;
      });
    windows_sub_ = create_subscription<dracoviloc_msgs::msg::ClassifierWindow>(
      "/mobilenetv2/diagnostics", reliable,
      [this](dracoviloc_msgs::msg::ClassifierWindow::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double now = wall();
        window_rate_.tick(now);
        inference_ms_ = inference_ms_ <= 0.0 ? msg->inference_ms : 0.9 * inference_ms_ + 0.1 * msg->inference_ms;
        auto & channel = channels_[msg->channel];
        channel.received = now;
        channel.track_id = msg->track_id;
        channel.activity = msg->activity;
        channel.probability = msg->probability;
        channel.votes = msg->votes;
        channel.vote_window = msg->vote_window;
        channel.decision = msg->decision;
        channel.actionable = msg->actionable;
      });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", sensor, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double t = msg->header.stamp.sec != 0 ? stamp_seconds(msg->header.stamp) : wall();
        for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          auto found = joints_.find(msg->name[i]);
          if (found == joints_.end()) {
            found = joints_.emplace(msg->name[i], JointDerivatives(joint_alpha_)).first;
          }
          found->second.update(msg->position[i], t);
        }
        joint_rate_.tick(wall());
      });
    for (const auto & [topic, target] : std::vector<std::pair<std::string, double *>>{
        {"/ee/speed", &ee_speed_}, {"/ee/acceleration", &ee_acceleration_}, {"/ee/jerk", &ee_jerk_}}) {
      ee_subs_.push_back(create_subscription<std_msgs::msg::Float64>(
        topic, 20, [this, target](std_msgs::msg::Float64::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          *target = msg->data;
          ee_received_ = wall();
        }));
    }
    sst_sub_ = create_subscription<odas_ros_msgs::msg::OdasSstArrayStamped>(
      "/sst", reliable, [this](odas_ros_msgs::msg::OdasSstArrayStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        sst_ = *msg;
        sst_rate_.tick(wall());
      });

    publisher_ = create_publisher<std_msgs::msg::String>("/dracoviloc/telemetry", 10);
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate), [this]() {publish();});
    log_every_ = std::max(1, static_cast<int>(std::lround(publish_rate / log_rate)));
  }

private:
  struct Direction
  {
    RateMeter rate;
    std::array<double, 3> vector{0.0, 0.0, 0.0};
    std::string frame;
  };
  struct EkfSource
  {
    RateMeter rate;
    std::map<std::string, int> decisions;
    std::string last_decision;
    double last_nis{NAN}, last_innovation_deg{NAN}, last_delay_s{NAN};
  };
  struct Channel
  {
    double received{-1.0};
    int track_id{0};
    double activity{0.0}, probability{0.0};
    int votes{0}, vote_window{0};
    bool decision{false}, actionable{false};
  };

  double wall() const {return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();}

  static void write_direction(JsonWriter & json, const std::array<double, 3> & v)
  {
    json.key("vector").begin_array().value(v[0]).value(v[1]).value(v[2]).end_array();
    json.field("azimuth_deg", std::atan2(v[1], v[0]) * kRadToDeg);
    json.field("elevation_deg", std::asin(std::clamp(v[2], -1.0, 1.0)) * kRadToDeg);
  }

  void publish()
  {
    std::string payload;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      payload = snapshot();
    }
    std_msgs::msg::String msg;
    msg.data = payload;
    publisher_->publish(msg);
    if (log_.is_open() && ++published_ % log_every_ == 0) {
      log_ << payload << '\n';
      log_.flush();
    }
  }

  std::string snapshot()
  {
    const double now = wall();
    const auto age = [now](double last) {return last < 0.0 ? NAN : now - last;};
    JsonWriter json;
    json.begin_object();
    json.field("schema", 1).field("t", now);

    json.key("camera").begin_object()
    .field("fps", camera_rate_.rate(now)).field("age_s", age(camera_rate_.last()))
    .field("width", camera_width_).field("height", camera_height_)
    .field("latency_ms", camera_latency_ * 1000.0).end_object();

    json.key("yolo").begin_object()
    .field("inference_rate_hz", detection_rate_.rate(now))
    .field("detection_rate_hz", positive_rate_.rate(now))
    .field("detections", detections_).field("best_score", best_score_)
    .field("latency_ms", yolo_latency_ * 1000.0)
    .field("age_s", age(detection_rate_.last()));
    json.end_object();

    json.key("directions").begin_object();
    for (auto & [name, direction] : directions_) {
      json.key(name).begin_object()
      .field("rate_hz", direction.rate.rate(now)).field("age_s", age(direction.rate.last()))
      .field("frame", direction.frame);
      write_direction(json, direction.vector);
      json.end_object();
    }
    json.end_object();

    json.key("mobilenetv2").begin_object()
    .field("window_rate_hz", window_rate_.rate(now)).field("inference_ms", inference_ms_)
    .field("age_s", age(window_rate_.last()));
    json.key("channels").begin_array();
    for (const auto & [index, channel] : channels_) {
      json.begin_object().field("channel", index).field("track_id", channel.track_id)
      .field("activity", channel.activity).field("probability", channel.probability)
      .field("votes", channel.votes).field("vote_window", channel.vote_window)
      .field("decision", channel.decision).field("actionable", channel.actionable)
      .field("age_s", age(channel.received)).end_object();
    }
    json.end_array().end_object();

    json.key("ekf").begin_object().field("nis_gate", ekf_nis_gate_);
    json.key("sources").begin_object();
    for (auto & [topic, source] : ekf_sources_) {
      json.key(topic).begin_object()
      .field("rate_hz", source.rate.rate(now)).field("age_s", age(source.rate.last()))
      .field("last_decision", source.last_decision).field("last_nis", source.last_nis)
      .field("last_innovation_deg", source.last_innovation_deg)
      .field("last_delay_ms", source.last_delay_s * 1000.0);
      json.key("decisions").begin_object();
      for (const auto & [decision, count] : source.decisions) {
        json.field(decision, count);
      }
      json.end_object().end_object();
    }
    json.end_object().end_object();

    json.key("joints").begin_object().field("rate_hz", joint_rate_.rate(now));
    json.key("items").begin_array();
    for (const auto & [name, joint] : joints_) {
      json.begin_object().field("name", name).field("position", joint.position())
      .field("velocity", joint.velocity()).field("acceleration", joint.acceleration())
      .field("jerk", joint.jerk()).end_object();
    }
    json.end_array().end_object();

    json.key("ee").begin_object().field("speed", ee_speed_)
    .field("acceleration", ee_acceleration_).field("jerk", ee_jerk_)
    .field("age_s", age(ee_received_)).end_object();

    json.key("audio").begin_object().field("rate_hz", sst_rate_.rate(now))
    .field("frame", sst_.header.frame_id);
    json.key("sources").begin_array();
    for (const auto & source : sst_.sources) {
      if (source.id == 0) {
        continue;
      }
      json.begin_object().field("id", static_cast<double>(source.id))
      .field("activity", source.activity);
      write_direction(json, {source.x, source.y, source.z});
      json.end_object();
    }
    json.end_array().end_object();

    json.end_object();
    return json.str();
  }

  std::mutex mutex_;
  double joint_alpha_{0.3};
  RateMeter camera_rate_, detection_rate_, positive_rate_, window_rate_, joint_rate_, sst_rate_;
  int camera_width_{0}, camera_height_{0}, detections_{0};
  double camera_latency_{NAN}, yolo_latency_{NAN}, best_score_{0.0}, inference_ms_{0.0};
  double ekf_nis_gate_{NAN};
  double ee_speed_{NAN}, ee_acceleration_{NAN}, ee_jerk_{NAN}, ee_received_{-1.0};
  std::map<std::string, Direction> directions_;
  std::map<std::string, EkfSource> ekf_sources_;
  std::map<int, Channel> channels_;
  std::map<std::string, JointDerivatives> joints_;
  odas_ros_msgs::msg::OdasSstArrayStamped sst_;
  std::ofstream log_;
  int log_every_{5};
  uint64_t published_{0};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr> direction_subs_;
  rclcpp::Subscription<dracoviloc_msgs::msg::EkfUpdate>::SharedPtr ekf_sub_;
  rclcpp::Subscription<dracoviloc_msgs::msg::ClassifierWindow>::SharedPtr windows_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> ee_subs_;
  rclcpp::Subscription<odas_ros_msgs::msg::OdasSstArrayStamped>::SharedPtr sst_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryNode>());
  rclcpp::shutdown();
  return 0;
}
