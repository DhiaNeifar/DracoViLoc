#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace fs = std::filesystem;

// =============================================================================
// event_logger_node  -  timing and end-effector kinematics diagnostics
//
// Answers two questions that the control-loop node itself cannot answer about
// itself: when did each detector actually produce something, and what did the
// arm's end effector actually do in Cartesian space as a result? It never
// touches the control loop -- it only observes topics arm_tracker and the
// detectors already publish, plus /joint_states, so it is safe to run
// alongside real hardware or the mock digital twin with zero coupling.
//
// EVENTS
// ======
// /yolo/direction, /mobilenetv2/direction, and /ekf/direction are each
// timestamped at RECEIPT (this node's own now()), not at the sender's publish
// time: for same-machine ROS 2 topics this is accurate to well under a
// millisecond, which is far finer than anything else in this pipeline.
// /tracking/events is arm_tracker's own output and is relayed verbatim --
// it is the one event this node cannot observe indirectly, because "the
// tracker decided to steer toward a new solved target" is internal state.
// All four are normalized onto /diagnostics/events so one subscription (the
// live plot, or any future consumer) sees the whole picture.
//
// END-EFFECTOR KINEMATICS
// ========================
// Position comes from TF (base_frame -> ee_frame), which is exact: it is the
// same forward kinematics RViz and MoveIt use, computed by robot_state_publisher
// from /joint_states. Velocity, acceleration, and jerk are NOT exact -- they
// are successive finite differences of that position, which amplifies sensor
// noise at each derivative. On the mock arm (a perfect integrator) this is
// close to the commanded values already logged by arm_tracker's servo_log.
// On real hardware it will be noisier, especially jerk; smoothing_alpha below
// trades responsiveness for noise rejection the same way the rest of this
// codebase's exponential filters do (1.0 disables it).
// =============================================================================

class EventLoggerNode : public rclcpp::Node {
public:
  EventLoggerNode() : Node("event_logger_node") {
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    ee_frame_ = declare_parameter("ee_frame", std::string("wrist3_link"));
    smoothing_alpha_ = declare_parameter("smoothing_alpha", 0.3);
    events_log_path_ = declare_parameter("events_log_path", std::string(""));
    ee_kinematics_log_path_ = declare_parameter("ee_kinematics_log_path", std::string(""));
    if (smoothing_alpha_ <= 0.0 || smoothing_alpha_ > 1.0) {
      throw std::invalid_argument("smoothing_alpha must be in (0, 1]; 1.0 disables smoothing");
    }

    open_events_log();
    open_kinematics_log();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    events_pub_ = create_publisher<std_msgs::msg::String>("/diagnostics/events", 50);
    speed_pub_ = create_publisher<std_msgs::msg::Float64>("/ee/speed", 20);
    accel_pub_ = create_publisher<std_msgs::msg::Float64>("/ee/acceleration", 20);
    jerk_pub_ = create_publisher<std_msgs::msg::Float64>("/ee/jerk", 20);
    position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/ee/position", 20);

    yolo_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/yolo/direction", 10,
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr) {handle_event("yolo_in");});
    mobilenetv2_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/mobilenetv2/direction", 10,
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr) {
        handle_event("mobilenetv2_in");
      });
    ekf_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/ekf/direction", 10,
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr) {handle_event("ekf_out");});
    tracking_events_sub_ = create_subscription<std_msgs::msg::String>(
      "/tracking/events", 20,
      [this](std_msgs::msg::String::SharedPtr msg) {handle_event(msg->data);});
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {handle_joint_states(*msg);});

    RCLCPP_INFO(get_logger(),
      "Logging timing events and %s -> %s end-effector kinematics "
      "(smoothing_alpha=%.2f)",
      base_frame_.c_str(), ee_frame_.c_str(), smoothing_alpha_);
  }

private:
  using Vec3 = std::array<double, 3>;

  static Vec3 sub(const Vec3 & a, const Vec3 & b) {return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};}
  static Vec3 scale(const Vec3 & a, double s) {return {a[0] * s, a[1] * s, a[2] * s};}
  static double norm(const Vec3 & a) {return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);}
  static Vec3 blend(const Vec3 & raw, const Vec3 & previous, double alpha) {
    return {
      alpha * raw[0] + (1.0 - alpha) * previous[0],
      alpha * raw[1] + (1.0 - alpha) * previous[1],
      alpha * raw[2] + (1.0 - alpha) * previous[2]};
  }

  void open_events_log() {
    if (events_log_path_.empty()) {return;}
    const fs::path path(events_log_path_);
    if (!path.parent_path().empty()) {fs::create_directories(path.parent_path());}
    const bool write_header = !fs::exists(path) || fs::file_size(path) == 0;
    events_log_.open(path, std::ios::app);
    if (!events_log_) {throw std::runtime_error("cannot open events log: " + path.string());}
    if (write_header) {
      events_log_ << "receipt_sec,receipt_nanosec,event_type\n";
    }
    events_log_ << std::setprecision(17);
    events_log_.flush();
    RCLCPP_INFO(get_logger(), "Logging events to %s", path.c_str());
  }

  void open_kinematics_log() {
    if (ee_kinematics_log_path_.empty()) {return;}
    const fs::path path(ee_kinematics_log_path_);
    if (!path.parent_path().empty()) {fs::create_directories(path.parent_path());}
    const bool write_header = !fs::exists(path) || fs::file_size(path) == 0;
    ee_kinematics_log_.open(path, std::ios::app);
    if (!ee_kinematics_log_) {
      throw std::runtime_error("cannot open end-effector kinematics log: " + path.string());
    }
    if (write_header) {
      ee_kinematics_log_
        << "time_sec,x,y,z,"
        << "vx,vy,vz,speed,"
        << "ax,ay,az,acceleration,"
        << "jx,jy,jz,jerk\n";
    }
    ee_kinematics_log_ << std::setprecision(17);
    ee_kinematics_log_.flush();
    RCLCPP_INFO(get_logger(), "Logging end-effector kinematics to %s", path.c_str());
  }

  void handle_event(const std::string & event_type) {
    const auto stamp = now();
    if (events_log_.is_open()) {
      const int64_t ns = stamp.nanoseconds();
      events_log_ << ns / 1000000000LL << ',' << ns % 1000000000LL << ',' << event_type << '\n';
      events_log_.flush();
    }
    std_msgs::msg::String relay;
    relay.data = event_type;
    events_pub_->publish(relay);
  }

  void handle_joint_states(const sensor_msgs::msg::JointState &) {
    // Looking up the latest available transform (tf2::TimePointZero) is
    // robust the way arm_tracker.cpp already relies on elsewhere: it never
    // fails to extrapolation the way asking for an exact stamp occasionally
    // does while the TF broadcast is still catching up. The bug this used to
    // have was pairing that lookup with wall-clock receipt time (now()) for
    // differentiation: two independently jittering clocks, which a triple
    // finite difference (velocity->acceleration->jerk) amplifies into huge
    // spurious spikes. The fix is to use the resolved transform's OWN
    // returned stamp for dt, since that is the instant the position actually
    // describes, regardless of which instant was requested.
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "no %s -> %s transform yet: %s", base_frame_.c_str(), ee_frame_.c_str(), e.what());
      return;
    }
    const rclcpp::Time stamp(transform.header.stamp);
    const Vec3 position {
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z};

    geometry_msgs::msg::PointStamped position_msg;
    position_msg.header.stamp = stamp;
    position_msg.header.frame_id = base_frame_;
    position_msg.point.x = position[0];
    position_msg.point.y = position[1];
    position_msg.point.z = position[2];
    position_pub_->publish(position_msg);

    if (!have_position_) {
      previous_position_ = position;
      previous_time_ = stamp;
      have_position_ = true;
      return;
    }
    // A duplicate or near-duplicate /joint_states stamp (observed in practice
    // from some publishers) carries no real elapsed time; skip it rather than
    // clamp dt up to an artificial floor, which would fabricate a velocity
    // from whatever position noise exists between the two identical-looking
    // samples and then amplify it through the acceleration/jerk differences.
    const double raw_dt = (stamp - previous_time_).seconds();
    if (raw_dt < 2.0e-3) {return;}
    const double dt = std::min(raw_dt, 0.5);
    const Vec3 raw_velocity = scale(sub(position, previous_position_), 1.0 / dt);
    velocity_ = have_velocity_ ? blend(raw_velocity, velocity_, smoothing_alpha_) : raw_velocity;

    if (have_velocity_prev_) {
      const Vec3 raw_acceleration = scale(sub(velocity_, previous_velocity_), 1.0 / dt);
      acceleration_ = have_acceleration_ ?
        blend(raw_acceleration, acceleration_, smoothing_alpha_) : raw_acceleration;
      if (have_acceleration_prev_) {
        const Vec3 raw_jerk = scale(sub(acceleration_, previous_acceleration_), 1.0 / dt);
        jerk_ = have_jerk_ ? blend(raw_jerk, jerk_, smoothing_alpha_) : raw_jerk;
        have_jerk_ = true;
      }
      have_acceleration_ = true;
    }
    have_velocity_ = true;

    if (have_jerk_) {
      std_msgs::msg::Float64 speed_msg, accel_msg, jerk_msg;
      speed_msg.data = norm(velocity_);
      accel_msg.data = norm(acceleration_);
      jerk_msg.data = norm(jerk_);
      speed_pub_->publish(speed_msg);
      accel_pub_->publish(accel_msg);
      jerk_pub_->publish(jerk_msg);

      if (ee_kinematics_log_.is_open()) {
        ee_kinematics_log_
          << stamp.seconds() << ','
          << position[0] << ',' << position[1] << ',' << position[2] << ','
          << velocity_[0] << ',' << velocity_[1] << ',' << velocity_[2] << ',' << norm(velocity_) << ','
          << acceleration_[0] << ',' << acceleration_[1] << ',' << acceleration_[2] << ',' << norm(acceleration_) << ','
          << jerk_[0] << ',' << jerk_[1] << ',' << jerk_[2] << ',' << norm(jerk_) << '\n';
        ee_kinematics_log_.flush();
      }
    }

    previous_position_ = position;
    previous_time_ = stamp;
    if (have_velocity_) {previous_velocity_ = velocity_; have_velocity_prev_ = true;}
    if (have_acceleration_) {previous_acceleration_ = acceleration_; have_acceleration_prev_ = true;}
  }

  std::string base_frame_, ee_frame_;
  double smoothing_alpha_;
  std::string events_log_path_, ee_kinematics_log_path_;
  std::ofstream events_log_, ee_kinematics_log_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yolo_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mobilenetv2_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr ekf_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracking_events_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_, accel_pub_, jerk_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;

  bool have_position_{false};
  rclcpp::Time previous_time_{0, 0, RCL_ROS_TIME};
  Vec3 previous_position_{};
  Vec3 velocity_{}, previous_velocity_{};
  Vec3 acceleration_{}, previous_acceleration_{};
  Vec3 jerk_{};
  bool have_velocity_{false}, have_velocity_prev_{false};
  bool have_acceleration_{false}, have_acceleration_prev_{false};
  bool have_jerk_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
