#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <Eigen/Core>

namespace dracoviloc_ekf
{

struct BearingGateConfig
{
  bool enabled{true};
  double base_angle_deg{45.0};
  double max_angle_deg{70.0};
  double max_rate_deg_s{20.0};
  int confirmations{5};
  double confirmation_time_s{2.0};
  double challenger_tolerance_deg{15.0};
  double reacquire_timeout_s{10.0};
};

enum class BearingGateDecision {Accept, Reject, Reacquire};

struct BearingGateResult
{
  BearingGateDecision decision{BearingGateDecision::Accept};
  double residual_deg{0.0};
  double allowed_deg{180.0};
  int pending_count{0};
  bool after_source_gap{false};
};

class BearingContinuityGate
{
public:
  explicit BearingContinuityGate(const BearingGateConfig & config)
  : config_(config)
  {
    validate();
    challenger_cos_ = std::cos(to_radians(config_.challenger_tolerance_deg));
  }

  bool enabled() const {return config_.enabled;}

  BearingGateResult evaluate(
    const Eigen::Vector3d & measurement, const Eigen::Vector3d & prediction,
    double timestamp, double nis, double nis_threshold, bool allow_reacquire)
  {
    BearingGateResult result;
    if (!config_.enabled) {
      return result;
    }

    if (!std::isfinite(timestamp) || !measurement.allFinite() || !prediction.allFinite() ||
      measurement.norm() < 1e-9 || prediction.norm() < 1e-9)
    {
      result.decision = BearingGateDecision::Reject;
      return result;
    }

    const Eigen::Vector3d candidate = measurement.normalized();
    const Eigen::Vector3d expected = prediction.normalized();
    result.residual_deg = to_degrees(std::acos(
      std::clamp(candidate.dot(expected), -1.0, 1.0)));

    // Only forward-moving timestamps contribute evidence. This prevents
    // interleaved sensor callbacks from manufacturing a timeout or multiple
    // confirmations from the same ODAS frame.
    if (have_last_seen_ && timestamp < last_seen_time_) {
      result.decision = BearingGateDecision::Reject;
      result.pending_count = pending_count_;
      return result;
    }

    const bool source_gap = have_last_seen_ &&
      timestamp - last_seen_time_ > config_.reacquire_timeout_s;
    if (!have_last_seen_ || timestamp > last_seen_time_) {
      last_seen_time_ = timestamp;
      have_last_seen_ = true;
    }
    result.after_source_gap = source_gap;

    if (source_gap) {
      // Classifier publications are intentionally intermittent. A gap must
      // expire stale challenger evidence, never make the next arbitrary
      // direction trusted. Large post-gap changes still need confirmation.
      clear_pending();
    }

    double elapsed = 0.0;
    if (have_last_accepted_) {
      elapsed = std::max(0.0, timestamp - last_accepted_time_);
    }
    result.allowed_deg = std::min(
      config_.max_angle_deg,
      config_.base_angle_deg + config_.max_rate_deg_s * elapsed);

    // NIS contains the EKF prediction covariance and the source-specific
    // measurement covariance, so uncertainty can excuse a moderate excess
    // over the motion envelope. It must not excuse a jump beyond the absolute
    // angular cap: a high covariance or noisy source would otherwise make the
    // continuity gate ineffective precisely when a strong reflection appears.
    const bool beyond_hard_cap = result.residual_deg > config_.max_angle_deg;
    const bool statistically_inconsistent =
      std::isfinite(nis) && std::isfinite(nis_threshold) && nis > nis_threshold;
    const bool suspicious = beyond_hard_cap ||
      (result.residual_deg > result.allowed_deg && statistically_inconsistent);
    if (!suspicious) {
      clear_pending();
      result.decision = BearingGateDecision::Accept;
      return result;
    }

    if (!have_pending_ || candidate.dot(pending_direction_) < challenger_cos_) {
      pending_direction_ = candidate;
      pending_count_ = 1;
      pending_first_time_ = timestamp;
      pending_last_time_ = timestamp;
      have_pending_ = true;
    } else if (timestamp > pending_last_time_) {
      ++pending_count_;
      pending_last_time_ = timestamp;
      // Follow a slowly moving challenger without letting one noisy sample
      // pull the cluster outside its own consistency cone.
      pending_direction_ = (pending_direction_ + candidate).normalized();
    }

    result.pending_count = pending_count_;
    const bool confirmed = pending_count_ >= config_.confirmations &&
      pending_last_time_ - pending_first_time_ >= config_.confirmation_time_s;
    result.decision = confirmed && allow_reacquire ?
      BearingGateDecision::Reacquire : BearingGateDecision::Reject;
    return result;
  }

  void mark_accepted(const Eigen::Vector3d & direction, double timestamp)
  {
    if (!config_.enabled || !std::isfinite(timestamp) || !direction.allFinite() ||
      direction.norm() < 1e-9)
    {
      return;
    }
    last_accepted_time_ = timestamp;
    have_last_accepted_ = true;
    if (!have_last_seen_ || timestamp > last_seen_time_) {
      last_seen_time_ = timestamp;
      have_last_seen_ = true;
    }
    clear_pending();
  }

  int pending_count() const {return pending_count_;}

private:
  static constexpr double kPi = 3.14159265358979323846;
  static double to_radians(double degrees) {return degrees * kPi / 180.0;}
  static double to_degrees(double radians) {return radians * 180.0 / kPi;}

  void validate() const
  {
    if (!std::isfinite(config_.base_angle_deg) || config_.base_angle_deg <= 0.0 ||
      !std::isfinite(config_.max_angle_deg) ||
      config_.max_angle_deg < config_.base_angle_deg || config_.max_angle_deg > 180.0 ||
      !std::isfinite(config_.max_rate_deg_s) || config_.max_rate_deg_s < 0.0 ||
      config_.confirmations < 1 ||
      !std::isfinite(config_.confirmation_time_s) || config_.confirmation_time_s < 0.0 ||
      !std::isfinite(config_.challenger_tolerance_deg) ||
      config_.challenger_tolerance_deg <= 0.0 || config_.challenger_tolerance_deg > 180.0 ||
      !std::isfinite(config_.reacquire_timeout_s) || config_.reacquire_timeout_s <= 0.0)
    {
      throw std::invalid_argument("invalid bearing continuity gate configuration");
    }
  }

  void clear_pending()
  {
    have_pending_ = false;
    pending_count_ = 0;
    pending_first_time_ = 0.0;
    pending_last_time_ = 0.0;
  }

  BearingGateConfig config_;
  double challenger_cos_{1.0};
  bool have_last_seen_{false};
  double last_seen_time_{0.0};
  bool have_last_accepted_{false};
  double last_accepted_time_{0.0};
  bool have_pending_{false};
  Eigen::Vector3d pending_direction_{Eigen::Vector3d::UnitX()};
  int pending_count_{0};
  double pending_first_time_{0.0};
  double pending_last_time_{0.0};
};

}  // namespace dracoviloc_ekf
