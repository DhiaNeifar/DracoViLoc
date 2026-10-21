#include <cmath>

#include <gtest/gtest.h>

#include "dracoviloc_ekf/direct_path_gate.hpp"

namespace
{

using dracoviloc_ekf::DirectPathDecision;
using dracoviloc_ekf::DirectPathGate;
using dracoviloc_ekf::DirectPathGateConfig;

Eigen::Vector3d bearing(double degrees)
{
  constexpr double pi = 3.14159265358979323846;
  const double radians = degrees * pi / 180.0;
  return {std::cos(radians), std::sin(radians), 0.0};
}

TEST(DirectPathGate, RequiresEvidenceForInitialAcousticBearing)
{
  DirectPathGate gate(DirectPathGateConfig{});
  EXPECT_EQ(gate.evaluate(bearing(30.0), 1.0).decision, DirectPathDecision::RejectNoAnchor);
  gate.observe(bearing(32.0), 0.9, 0.5);
  EXPECT_EQ(
    gate.evaluate(bearing(30.0), 1.0).decision,
    DirectPathDecision::AllowMatchedEvidence);
}

TEST(DirectPathGate, AllowsContinuityNearValidatedAnchor)
{
  DirectPathGate gate(DirectPathGateConfig{});
  gate.observe(bearing(30.0), 0.9, 0.0);
  ASSERT_TRUE(gate.evaluate(bearing(30.0), 0.1).allowed());
  EXPECT_EQ(gate.evaluate(bearing(48.0), 3.0).decision, DirectPathDecision::AllowAnchored);
}

TEST(DirectPathGate, ReflectionCannotMoveBeyondAnchorWithoutEvidence)
{
  DirectPathGate gate(DirectPathGateConfig{});
  gate.mark_anchor(bearing(30.0), 0.0);
  EXPECT_EQ(
    gate.evaluate(bearing(125.0), 1.0).decision,
    DirectPathDecision::RejectOutsideAnchor);
}

TEST(DirectPathGate, FreshEvidenceAuthorizesAbruptRealMovement)
{
  DirectPathGate gate(DirectPathGateConfig{});
  gate.mark_anchor(bearing(30.0), 0.0);
  gate.observe(bearing(123.0), 0.8, 4.0);
  EXPECT_EQ(
    gate.evaluate(bearing(125.0), 4.5).decision,
    DirectPathDecision::AllowMatchedEvidence);
}

TEST(DirectPathGate, StaleAndLowConfidenceEvidenceCannotAuthorize)
{
  DirectPathGate gate(DirectPathGateConfig{});
  gate.observe(bearing(125.0), 0.6, 0.0);
  EXPECT_EQ(gate.evaluate(bearing(125.0), 0.5).decision, DirectPathDecision::RejectNoAnchor);
  gate.observe(bearing(125.0), 0.9, 1.0);
  EXPECT_EQ(gate.evaluate(bearing(125.0), 3.1).decision, DirectPathDecision::RejectNoAnchor);
}

TEST(DirectPathGate, AngularComparisonHandlesWraparound)
{
  DirectPathGate gate(DirectPathGateConfig{});
  gate.mark_anchor(bearing(179.0), 0.0);
  EXPECT_EQ(gate.evaluate(bearing(-179.0), 1.0).decision, DirectPathDecision::AllowAnchored);
}

}  // namespace
