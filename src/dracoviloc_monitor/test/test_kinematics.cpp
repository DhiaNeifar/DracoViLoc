#include <gtest/gtest.h>

#include <cmath>

#include "dracoviloc_monitor/monitor_utils.hpp"

using dracoviloc_monitor::JointDerivatives;
using dracoviloc_monitor::JsonWriter;
using dracoviloc_monitor::RateMeter;

TEST(JointDerivatives, ConstantAccelerationIsRecovered)
{
  JointDerivatives joint(1.0);  // no smoothing: exact finite differences
  for (int i = 0; i <= 100; ++i) {
    const double t = i * 0.01;
    joint.update(0.5 * 2.0 * t * t, t);  // a = 2 rad/s^2
  }
  EXPECT_NEAR(joint.velocity(), 2.0, 0.02);
  EXPECT_NEAR(joint.acceleration(), 2.0, 1e-6);
  EXPECT_NEAR(joint.jerk(), 0.0, 1e-6);
}

TEST(JointDerivatives, DuplicateTimestampsAreIgnored)
{
  JointDerivatives joint(1.0);
  joint.update(0.0, 1.0);
  joint.update(1.0, 1.0);
  EXPECT_EQ(joint.velocity(), 0.0);
}

TEST(RateMeter, MeasuresSteadyRate)
{
  RateMeter meter(2.0);
  for (int i = 0; i < 100; ++i) {
    meter.tick(i * 0.1);
  }
  EXPECT_NEAR(meter.rate(9.9), 10.0, 0.6);
  EXPECT_EQ(meter.rate(100.0), 0.0);
}

TEST(JsonWriter, NestedStructureAndEscaping)
{
  JsonWriter json;
  json.begin_object().field("a", 1).key("b").begin_array().value(1.5).value("x\"y").end_array()
  .key("c").begin_object().field("ok", true).end_object().field("n", NAN).end_object();
  EXPECT_EQ(json.str(), R"({"a":1,"b":[1.5,"x\"y"],"c":{"ok":true},"n":null})");
}
