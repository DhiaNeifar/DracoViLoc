#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>

#include <Eigen/Core>

namespace dracoviloc_ekf
{

struct DirectPathGateConfig
{
  double min_confidence{0.70};
  double match_angle_deg{20.0};
  double anchor_max_deviation_deg{25.0};
  double evidence_ttl_s{2.0};
};

enum class DirectPathDecision {AllowAnchored, AllowMatchedEvidence, RejectNoAnchor, RejectOutsideAnchor};

struct DirectPathResult
{
  DirectPathDecision decision{DirectPathDecision::RejectNoAnchor};
  double anchor_residual_deg{180.0};
  double evidence_residual_deg{180.0};

  bool allowed() const
  {
    return decision == DirectPathDecision::AllowAnchored ||
           decision == DirectPathDecision::AllowMatchedEvidence;
  }
};

class DirectPathGate
{
public:
  explicit DirectPathGate(const DirectPathGateConfig & config)
  : config_(config)
  {
    if (!std::isfinite(config_.min_confidence) || config_.min_confidence < 0.0 ||
      config_.min_confidence > 1.0 || !valid_angle(config_.match_angle_deg) ||
      !valid_angle(config_.anchor_max_deviation_deg) ||
      !std::isfinite(config_.evidence_ttl_s) || config_.evidence_ttl_s <= 0.0)
    {
      throw std::invalid_argument("invalid direct path gate configuration");
    }
  }

  void observe(const Eigen::Vector3d & direction, double confidence, double timestamp)
  {
    if (!direction.allFinite() || direction.norm() < 1.0e-9 ||
      !std::isfinite(confidence) || confidence < config_.min_confidence ||
      !std::isfinite(timestamp))
    {
      return;
    }
    evidence_.push_back({direction.normalized(), confidence, timestamp});
    while (evidence_.size() > 32) {
      evidence_.pop_front();
    }
  }

  void mark_anchor(const Eigen::Vector3d & direction, double timestamp)
  {
    if (!direction.allFinite() || direction.norm() < 1.0e-9 || !std::isfinite(timestamp)) {
      return;
    }
    anchor_ = direction.normalized();
    anchor_time_ = timestamp;
    have_anchor_ = true;
  }

  DirectPathResult evaluate(const Eigen::Vector3d & measurement, double timestamp)
  {
    DirectPathResult result;
    if (!measurement.allFinite() || measurement.norm() < 1.0e-9 || !std::isfinite(timestamp)) {
      return result;
    }
    const Eigen::Vector3d candidate = measurement.normalized();
    prune(timestamp);

    const Evidence * matching = nullptr;
    for (auto iterator = evidence_.rbegin(); iterator != evidence_.rend(); ++iterator) {
      if (iterator->timestamp > timestamp + 0.1) {
        continue;
      }
      const double residual = angle_deg(candidate, iterator->direction);
      if (residual < result.evidence_residual_deg) {
        result.evidence_residual_deg = residual;
      }
      if (residual <= config_.match_angle_deg) {
        matching = &*iterator;
        break;
      }
    }

    if (matching != nullptr) {
      mark_anchor(matching->direction, matching->timestamp);
      result.decision = DirectPathDecision::AllowMatchedEvidence;
      result.anchor_residual_deg = angle_deg(candidate, anchor_);
      return result;
    }
    if (!have_anchor_) {
      result.decision = DirectPathDecision::RejectNoAnchor;
      return result;
    }
    result.anchor_residual_deg = angle_deg(candidate, anchor_);
    result.decision = result.anchor_residual_deg <= config_.anchor_max_deviation_deg ?
      DirectPathDecision::AllowAnchored : DirectPathDecision::RejectOutsideAnchor;
    return result;
  }

  bool have_anchor() const {return have_anchor_;}

private:
  struct Evidence
  {
    Eigen::Vector3d direction;
    double confidence;
    double timestamp;
  };

  static constexpr double kPi = 3.14159265358979323846;

  static bool valid_angle(double angle)
  {
    return std::isfinite(angle) && angle > 0.0 && angle <= 180.0;
  }

  static double angle_deg(const Eigen::Vector3d & first, const Eigen::Vector3d & second)
  {
    return std::acos(std::clamp(first.dot(second), -1.0, 1.0)) * 180.0 / kPi;
  }

  void prune(double timestamp)
  {
    while (!evidence_.empty() && timestamp - evidence_.front().timestamp > config_.evidence_ttl_s) {
      evidence_.pop_front();
    }
  }

  DirectPathGateConfig config_;
  std::deque<Evidence> evidence_;
  bool have_anchor_{false};
  Eigen::Vector3d anchor_{Eigen::Vector3d::UnitX()};
  double anchor_time_{0.0};
};

}  // namespace dracoviloc_ekf
