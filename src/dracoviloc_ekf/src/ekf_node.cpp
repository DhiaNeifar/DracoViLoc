#include <algorithm>
#include <cmath>
#include <limits>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include "dracoviloc_ekf/bearing_continuity_gate.hpp"
#include "dracoviloc_ekf/direct_path_gate.hpp"
#include "dracoviloc_msgs/msg/ekf_update.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "odas_ros_msgs/msg/odas_ssl_array_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class DirectionEkf : public rclcpp::Node {
public:
  DirectionEkf() : Node("dracoviloc_ekf") {
    frame_ = declare_parameter("tracking_frame", "table_mic_link");
    q_ = declare_parameter("process_noise", 0.05);
    r_ = declare_parameter("measurement_noise", 0.02);
    gate_ = declare_parameter("innovation_gate", 11.34);
    output_average_window_ = declare_parameter("output_average_window", 5);
    if (output_average_window_ < 1 || output_average_window_ > 100) {
      throw std::invalid_argument("output_average_window must be between 1 and 100");
    }
    bearing_gate_config_.enabled = declare_parameter("bearing_gate_enabled", true);
    bearing_gate_config_.base_angle_deg =
      declare_parameter("bearing_gate_base_angle_deg", 45.0);
    bearing_gate_config_.max_angle_deg =
      declare_parameter("bearing_gate_max_angle_deg", 70.0);
    bearing_gate_config_.max_rate_deg_s =
      declare_parameter("bearing_gate_max_rate_deg_s", 20.0);
    bearing_gate_config_.confirmations =
      declare_parameter("bearing_gate_confirmations", 5);
    bearing_gate_config_.confirmation_time_s =
      declare_parameter("bearing_gate_confirmation_time_s", 2.0);
    bearing_gate_config_.challenger_tolerance_deg =
      declare_parameter("bearing_gate_challenger_tolerance_deg", 15.0);
    bearing_gate_config_.reacquire_timeout_s =
      declare_parameter("bearing_gate_reacquire_timeout_s", 10.0);
    bearing_gate_ = std::make_unique<dracoviloc_ekf::BearingContinuityGate>(
      bearing_gate_config_);
    direct_path_mode_ = declare_parameter("first_arrival_mode", "off");
    if (direct_path_mode_ != "off" && direct_path_mode_ != "shadow" &&
      direct_path_mode_ != "enforce")
    {
      throw std::invalid_argument("first_arrival_mode must be off, shadow, or enforce");
    }
    direct_path_config_.min_confidence =
      declare_parameter("first_arrival_min_confidence", 0.70);
    direct_path_config_.match_angle_deg =
      declare_parameter("first_arrival_match_angle_deg", 20.0);
    direct_path_config_.anchor_max_deviation_deg =
      declare_parameter("first_arrival_anchor_deviation_deg", 25.0);
    direct_path_config_.evidence_ttl_s =
      declare_parameter("first_arrival_evidence_ttl_s", 2.0);
    direct_path_gate_ = std::make_unique<dracoviloc_ekf::DirectPathGate>(direct_path_config_);
    if (direct_path_mode_ != "off") {
      first_arrival_sub_ = create_subscription<odas_ros_msgs::msg::OdasSslArrayStamped>(
        "/first_arrival/candidates", 10,
        [this](odas_ros_msgs::msg::OdasSslArrayStamped::SharedPtr message) {
          const double timestamp = rclcpp::Time(message->header.stamp).seconds();
          for (const auto & source : message->sources) {
            direct_path_gate_->observe(
              Eigen::Vector3d(source.x, source.y, source.z), source.e, timestamp);
          }
        });
    }
    pub_ = create_publisher<Msg>("/ekf/direction", 10);
    legacy_pub_ = create_publisher<Msg>("/ekf_fused_target_pose", 10);
    diagnostics_pub_ = create_publisher<dracoviloc_msgs::msg::EkfUpdate>("/ekf/diagnostics", 50);
    add_source("yolo_enabled", "/yolo/direction", "yolo_measurement_noise", false, yolo_);
    add_source(
      "mobilenetv2_ekf_enabled", "/mobilenetv2/direction",
      "mobilenetv2_measurement_noise", true, mobilenetv2_);
    x_.setZero(); P_.setIdentity(); P_ *= 0.5;
    RCLCPP_INFO(
      get_logger(),
      "bearing continuity gate %s (base=%.1fdeg hard=%.1fdeg rate=%.1fdeg/s, "
      "reacquire=%d samples over %.2fs)",
      bearing_gate_config_.enabled ? "enabled" : "disabled",
      bearing_gate_config_.base_angle_deg, bearing_gate_config_.max_angle_deg,
      bearing_gate_config_.max_rate_deg_s, bearing_gate_config_.confirmations,
      bearing_gate_config_.confirmation_time_s);
    RCLCPP_INFO(
      get_logger(),
      "first-arrival gate mode=%s (confidence>=%.2f match=%.1fdeg anchor=%.1fdeg ttl=%.1fs)",
      direct_path_mode_.c_str(), direct_path_config_.min_confidence,
      direct_path_config_.match_angle_deg, direct_path_config_.anchor_max_deviation_deg,
      direct_path_config_.evidence_ttl_s);
  }
private:
  using Msg = geometry_msgs::msg::Vector3Stamped;
  using Sub = rclcpp::Subscription<Msg>::SharedPtr;
  void add_source(
    const char * parameter, const char * topic, const char * noise_parameter,
    bool acoustic, Sub & sub)
  {
    const double configured_noise = declare_parameter(noise_parameter, -1.0);
    const double source_noise = configured_noise > 0.0 ? configured_noise : r_;
    if (!declare_parameter(parameter, false)) return;
    sub = create_subscription<Msg>(
      topic, 10,
      [this, topic, source_noise, acoustic](Msg::SharedPtr m) {
        update(*m, topic, source_noise, acoustic);
      });
    RCLCPP_INFO(
      get_logger(), "listening to %s (measurement_noise=%.4f)", topic, source_noise);
  }
  void predict(double t) {
    if (!ready_) return;
    const double dt = std::clamp(t - last_t_, 0.0, 0.5);
    Eigen::Matrix<double,6,6> F = Eigen::Matrix<double,6,6>::Identity();
    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
    x_ = F * x_; P_ = F * P_ * F.transpose() + Eigen::Matrix<double,6,6>::Identity() * q_ * dt;
    // Sensor callbacks can be interleaved and their header timestamps need not
    // arrive in order. Never move the filter clock backwards.
    last_t_ = std::max(last_t_, t);
  }
  void update(
    const Msg & msg, const char * source, double measurement_noise, bool acoustic)
  {
    Eigen::Vector3d z(msg.vector.x, msg.vector.y, msg.vector.z);
    if (!z.allFinite() || z.norm() < 1e-6) return;
    z.normalize();
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    if (!std::isfinite(t)) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // Reports every outcome on /ekf/diagnostics; never affects the filter.
    const auto report = [&](const char * decision, double nis, double innovation_deg) {
      dracoviloc_msgs::msg::EkfUpdate d;
      d.header.stamp = msg.header.stamp;
      d.header.frame_id = frame_;
      d.source = source;
      d.decision = decision;
      d.nis = nis;
      d.nis_gate = gate_;
      d.innovation_deg = innovation_deg;
      d.measurement_noise = measurement_noise;
      d.processing_delay_s = now().seconds() - t;
      d.measurement.x = z.x(); d.measurement.y = z.y(); d.measurement.z = z.z();
      const Eigen::Vector3d estimate = x_.head<3>().norm() > 1e-9
        ? Eigen::Vector3d(x_.head<3>().normalized()) : Eigen::Vector3d::Zero();
      d.estimate.x = estimate.x(); d.estimate.y = estimate.y(); d.estimate.z = estimate.z();
      diagnostics_pub_->publish(d);
    };
    if (acoustic && direct_path_mode_ != "off") {
      const auto direct_result = direct_path_gate_->evaluate(z, t);
      if (!direct_result.allowed()) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "first-arrival gate %s %s: anchor_residual=%.1fdeg evidence_residual=%.1fdeg",
          direct_path_mode_ == "enforce" ? "rejected" : "would reject",
          source, direct_result.anchor_residual_deg, direct_result.evidence_residual_deg);
        if (direct_path_mode_ == "enforce") {
          report("rejected_first_arrival", nan, nan);
          return;
        }
      }
    }
    if (!ready_) {
      x_.head<3>() = z;
      last_t_ = t;
      ready_ = true;
      if (acoustic) {
        bearing_gate_->mark_accepted(z, t);
      } else {
        last_visual_update_t_ = t;
        have_visual_update_ = true;
        direct_path_gate_->mark_anchor(z, t);
      }
      publish(msg.header.stamp);
      report("initialized", nan, 0.0);
      return;
    }
    predict(t);
    Eigen::Matrix<double,3,6> H = Eigen::Matrix<double,3,6>::Zero(); H.block<3,3>(0,0).setIdentity();
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * measurement_noise;
    const Eigen::Vector3d innovation = z - H*x_;
    const Eigen::Matrix3d S = H*P_*H.transpose()+R;
    const double nis = innovation.transpose()*S.inverse()*innovation;
    const double innovation_deg = std::acos(std::clamp(
      z.dot(x_.head<3>().normalized()), -1.0, 1.0)) * 180.0 / M_PI;
    if (acoustic && bearing_gate_->enabled()) {
      const bool visual_is_fresh = have_visual_update_ &&
        std::abs(t - last_visual_update_t_) <= bearing_gate_config_.reacquire_timeout_s;
      const auto result = bearing_gate_->evaluate(
        z, x_.head<3>(), t, nis, gate_, !visual_is_fresh);
      if (result.decision == dracoviloc_ekf::BearingGateDecision::Reject) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "bearing gate rejected %s: residual=%.1fdeg allowed=%.1fdeg "
          "nis=%.2f pending=%d%s",
          source, result.residual_deg, result.allowed_deg, nis, result.pending_count,
          visual_is_fresh ? " (visual anchor active)" : "");
        report("rejected_bearing_gate", nis, innovation_deg);
        return;
      }
      if (result.decision == dracoviloc_ekf::BearingGateDecision::Reacquire) {
        x_.setZero();
        x_.head<3>() = z;
        P_.setIdentity();
        P_ *= 0.5;
        last_t_ = std::max(last_t_, t);
        output_history_.clear();
        bearing_gate_->mark_accepted(z, t);
        RCLCPP_WARN(
          get_logger(),
          "bearing gate reacquired %s at residual %.1fdeg%s",
          source, result.residual_deg,
          result.after_source_gap ? " after source loss" : " after confirmation");
        publish(msg.header.stamp);
        report("reacquired", nis, innovation_deg);
        return;
      }
    }
    if (nis > gate_) {
      RCLCPP_DEBUG(get_logger(), "rejected %s measurement", source);
      report("rejected_nis", nis, innovation_deg);
      return;
    }
    const Eigen::Matrix<double,6,3> K = P_*H.transpose()*S.inverse();
    x_ += K*innovation; P_ = (Eigen::Matrix<double,6,6>::Identity()-K*H)*P_;
    x_.head<3>().normalize();
    if (acoustic) {
      bearing_gate_->mark_accepted(z, t);
    } else {
      last_visual_update_t_ = std::max(last_visual_update_t_, t);
      have_visual_update_ = true;
      direct_path_gate_->mark_anchor(z, t);
    }
    publish(msg.header.stamp);
    report("accepted", nis, innovation_deg);
  }
  void publish(const builtin_interfaces::msg::Time & stamp) {
    output_history_.push_back(x_.head<3>().normalized());
    while (output_history_.size() > static_cast<std::size_t>(output_average_window_)) {
      output_history_.pop_front();
    }
    Eigen::Vector3d averaged = Eigen::Vector3d::Zero();
    for (const auto & direction : output_history_) averaged += direction;
    if (averaged.norm() < 1e-6) {
      averaged = x_.head<3>().normalized();
    } else {
      averaged.normalize();
    }
    Msg out; out.header.stamp=stamp; out.header.frame_id=frame_;
    out.vector.x=averaged.x(); out.vector.y=averaged.y(); out.vector.z=averaged.z();
    pub_->publish(out);
    legacy_pub_->publish(out);
  }
  std::string frame_; double q_,r_,gate_,last_t_{0}; bool ready_{false};
  dracoviloc_ekf::BearingGateConfig bearing_gate_config_;
  std::unique_ptr<dracoviloc_ekf::BearingContinuityGate> bearing_gate_;
  dracoviloc_ekf::DirectPathGateConfig direct_path_config_;
  std::unique_ptr<dracoviloc_ekf::DirectPathGate> direct_path_gate_;
  std::string direct_path_mode_{"off"};
  double last_visual_update_t_{0.0};
  bool have_visual_update_{false};
  int output_average_window_{5};
  std::deque<Eigen::Vector3d> output_history_;
  Eigen::Matrix<double,6,1> x_; Eigen::Matrix<double,6,6> P_;
  rclcpp::Publisher<Msg>::SharedPtr pub_, legacy_pub_; Sub yolo_,mobilenetv2_;
  rclcpp::Publisher<dracoviloc_msgs::msg::EkfUpdate>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<odas_ros_msgs::msg::OdasSslArrayStamped>::SharedPtr first_arrival_sub_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<DirectionEkf>());rclcpp::shutdown();}
