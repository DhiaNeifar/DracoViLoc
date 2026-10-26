// Small, dependency-free helpers shared by the monitor nodes (and unit tests).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>

namespace dracoviloc_monitor
{

// Minimal JSON writer: enough for flat telemetry snapshots, no allocations
// beyond the output string. Non-finite numbers are written as null.
class JsonWriter
{
public:
  JsonWriter & begin_object() {return open('{');}
  JsonWriter & end_object() {return close('}');}
  JsonWriter & begin_array() {return open('[');}
  JsonWriter & end_array() {return close(']');}

  JsonWriter & key(const std::string & name)
  {
    separator();
    string_literal(name);
    out_ += ':';
    after_key_ = true;
    return *this;
  }

  JsonWriter & value(double number)
  {
    separator();
    if (!std::isfinite(number)) {
      out_ += "null";
    } else {
      char buffer[32];
      std::snprintf(buffer, sizeof(buffer), "%.6g", number);
      out_ += buffer;
    }
    return *this;
  }
  JsonWriter & value(int number) {return value(static_cast<double>(number));}
  JsonWriter & value(bool flag)
  {
    separator();
    out_ += flag ? "true" : "false";
    return *this;
  }
  JsonWriter & value(const std::string & text)
  {
    separator();
    string_literal(text);
    return *this;
  }
  JsonWriter & value(const char * text) {return value(std::string(text));}

  template<typename T>
  JsonWriter & field(const std::string & name, const T & data) {return key(name).value(data);}

  const std::string & str() const {return out_;}

private:
  JsonWriter & open(char bracket)
  {
    separator();
    out_ += bracket;
    first_ = true;
    return *this;
  }
  JsonWriter & close(char bracket)
  {
    out_ += bracket;
    first_ = false;
    return *this;
  }
  void separator()
  {
    if (after_key_) {
      after_key_ = false;
      return;
    }
    if (!first_ && !out_.empty()) {
      out_ += ',';
    }
    first_ = false;
  }
  void string_literal(const std::string & text)
  {
    out_ += '"';
    for (char c : text) {
      switch (c) {
        case '"': out_ += "\\\""; break;
        case '\\': out_ += "\\\\"; break;
        case '\n': out_ += "\\n"; break;
        default:
          if (static_cast<unsigned char>(c) < 0x20) {
            char buffer[8];
            std::snprintf(buffer, sizeof(buffer), "\\u%04x", c);
            out_ += buffer;
          } else {
            out_ += c;
          }
      }
    }
    out_ += '"';
  }

  std::string out_;
  bool first_{true};
  bool after_key_{false};
};

// Message rate over a sliding window of receive times.
class RateMeter
{
public:
  RateMeter()
  : window_s_(2.0) {}
  explicit RateMeter(double window_s)
  : window_s_(window_s) {}

  void tick(double now)
  {
    times_.push_back(now);
    trim(now);
  }
  double rate(double now)
  {
    trim(now);
    if (times_.size() < 2) {
      return 0.0;
    }
    const double span = now - times_.front();
    return span > 0.0 ? static_cast<double>(times_.size()) / std::max(span, window_s_ * 0.5) : 0.0;
  }
  double last() const {return times_.empty() ? -1.0 : times_.back();}

private:
  void trim(double now)
  {
    while (!times_.empty() && now - times_.front() > window_s_) {
      times_.pop_front();
    }
  }
  double window_s_;
  std::deque<double> times_;
};

// Velocity, acceleration and jerk of one joint from successive positions,
// each stage low-pass filtered so finite differences of a 100 Hz signal stay
// readable. alpha = weight of the newest sample (0 < alpha <= 1).
class JointDerivatives
{
public:
  explicit JointDerivatives(double alpha = 0.3)
  : alpha_(alpha) {}

  void update(double position, double t)
  {
    if (have_position_ && t > last_t_) {
      const double dt = t - last_t_;
      const double velocity = blend(velocity_, (position - position_) / dt, have_velocity_);
      if (have_velocity_) {
        const double acceleration = blend(acceleration_, (velocity - velocity_) / dt, have_acceleration_);
        if (have_acceleration_) {
          jerk_ = blend(jerk_, (acceleration - acceleration_) / dt, have_jerk_);
          have_jerk_ = true;
        }
        acceleration_ = acceleration;
        have_acceleration_ = true;
      }
      velocity_ = velocity;
      have_velocity_ = true;
    }
    position_ = position;
    last_t_ = t;
    have_position_ = true;
  }

  double position() const {return position_;}
  double velocity() const {return have_velocity_ ? velocity_ : 0.0;}
  double acceleration() const {return have_acceleration_ ? acceleration_ : 0.0;}
  double jerk() const {return have_jerk_ ? jerk_ : 0.0;}

private:
  double blend(double previous, double sample, bool have_previous) const
  {
    return have_previous ? previous + alpha_ * (sample - previous) : sample;
  }
  double alpha_;
  double position_{0.0}, velocity_{0.0}, acceleration_{0.0}, jerk_{0.0}, last_t_{0.0};
  bool have_position_{false}, have_velocity_{false}, have_acceleration_{false}, have_jerk_{false};
};

}  // namespace dracoviloc_monitor
