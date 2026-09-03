#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class DirectionEkf : public rclcpp::Node {
public:
  DirectionEkf() : Node("dracoviloc_ekf") {
    frame_ = declare_parameter("tracking_frame", "table_mic_link");
    q_ = declare_parameter("process_noise", 0.05);
    r_ = declare_parameter("measurement_noise", 0.02);
    gate_ = declare_parameter("innovation_gate", 11.34);
    pub_ = create_publisher<Msg>("/ekf_fused_target_pose", 10);
    add_source("yolo_enabled", "/yolo/direction", yolo_);
    add_source("ast_enabled", "/ast/direction", ast_);
    add_source("gre_enabled", "/gre/direction", gre_);
    x_.setZero(); P_.setIdentity(); P_ *= 0.5;
  }
private:
  using Msg = geometry_msgs::msg::Vector3Stamped;
  using Sub = rclcpp::Subscription<Msg>::SharedPtr;
  void add_source(const char * parameter, const char * topic, Sub & sub) {
    if (!declare_parameter(parameter, false)) return;
    sub = create_subscription<Msg>(topic, 10, [this, topic](Msg::SharedPtr m) { update(*m, topic); });
    RCLCPP_INFO(get_logger(), "listening to %s", topic);
  }
  void predict(double t) {
    if (!ready_) return;
    const double dt = std::clamp(t - last_t_, 0.0, 0.5);
    Eigen::Matrix<double,6,6> F = Eigen::Matrix<double,6,6>::Identity();
    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
    x_ = F * x_; P_ = F * P_ * F.transpose() + Eigen::Matrix<double,6,6>::Identity() * q_ * dt;
    last_t_ = t;
  }
  void update(const Msg & msg, const char * source) {
    Eigen::Vector3d z(msg.vector.x, msg.vector.y, msg.vector.z);
    if (z.norm() < 1e-6) return;
    z.normalize();
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    if (!ready_) { x_.head<3>() = z; last_t_ = t; ready_ = true; publish(msg.header.stamp); return; }
    predict(t);
    Eigen::Matrix<double,3,6> H = Eigen::Matrix<double,3,6>::Zero(); H.block<3,3>(0,0).setIdentity();
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * r_;
    const Eigen::Vector3d innovation = z - H*x_;
    const Eigen::Matrix3d S = H*P_*H.transpose()+R;
    const double nis = innovation.transpose()*S.inverse()*innovation;
    if (nis > gate_) { RCLCPP_DEBUG(get_logger(), "rejected %s measurement", source); return; }
    const Eigen::Matrix<double,6,3> K = P_*H.transpose()*S.inverse();
    x_ += K*innovation; P_ = (Eigen::Matrix<double,6,6>::Identity()-K*H)*P_;
    x_.head<3>().normalize(); publish(msg.header.stamp);
  }
  void publish(const builtin_interfaces::msg::Time & stamp) {
    Msg out; out.header.stamp=stamp; out.header.frame_id=frame_;
    out.vector.x=x_(0); out.vector.y=x_(1); out.vector.z=x_(2); pub_->publish(out);
  }
  std::string frame_; double q_,r_,gate_,last_t_{0}; bool ready_{false};
  Eigen::Matrix<double,6,1> x_; Eigen::Matrix<double,6,6> P_;
  rclcpp::Publisher<Msg>::SharedPtr pub_; Sub yolo_,ast_,gre_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<DirectionEkf>());rclcpp::shutdown();}
