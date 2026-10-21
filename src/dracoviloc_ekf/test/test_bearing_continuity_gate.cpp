#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "dracoviloc_ekf/bearing_continuity_gate.hpp"

namespace
{

using dracoviloc_ekf::BearingContinuityGate;
using dracoviloc_ekf::BearingGateConfig;
using dracoviloc_ekf::BearingGateDecision;

Eigen::Vector3d bearing(double degrees)
{
  constexpr double pi = 3.14159265358979323846;
  const double radians = degrees * pi / 180.0;
  return {std::cos(radians), std::sin(radians), 0.0};
}

BearingContinuityGate seeded_gate(
  const BearingGateConfig & config = BearingGateConfig{})
{
  BearingContinuityGate gate(config);
  gate.mark_accepted(bearing(30.0), 0.0);
  return gate;
}

TEST(BearingContinuityGate, RejectsSuddenReflection)
{
  auto gate = seeded_gate();
  auto gradual = gate.evaluate(bearing(32.0), bearing(30.0), 0.1, 1.0, 11.34, true);
  ASSERT_EQ(gradual.decision, BearingGateDecision::Accept);
  gate.mark_accepted(bearing(32.0), 0.1);

  const auto reflection = gate.evaluate(
    bearing(125.0), bearing(32.0), 0.2, 50.0, 11.34, true);
  EXPECT_EQ(reflection.decision, BearingGateDecision::Reject);
  EXPECT_NEAR(reflection.residual_deg, 93.0, 1e-9);
  EXPECT_EQ(reflection.pending_count, 1);
}

TEST(BearingContinuityGate, AcceptsGradualDirectionChanges)
{
  auto gate = seeded_gate();
  double previous = 30.0;
  for (int index = 1; index <= 10; ++index) {
    const double next = 30.0 + index * 5.0;
    const double stamp = index * 0.1;
    const auto result = gate.evaluate(
      bearing(next), bearing(previous), stamp, 2.0, 11.34, true);
    ASSERT_EQ(result.decision, BearingGateDecision::Accept);
    gate.mark_accepted(bearing(next), stamp);
    previous = next;
  }
}

TEST(BearingContinuityGate, HandlesAzimuthWraparound)
{
  BearingContinuityGate gate(BearingGateConfig{});
  gate.mark_accepted(bearing(179.0), 0.0);
  const auto result = gate.evaluate(
    bearing(-179.0), bearing(179.0), 0.1, 20.0, 11.34, true);
  EXPECT_EQ(result.decision, BearingGateDecision::Accept);
  EXPECT_NEAR(result.residual_deg, 2.0, 1e-9);
}

TEST(BearingContinuityGate, AllowsRapidMovementUsingElapsedTime)
{
  BearingGateConfig config;
  config.base_angle_deg = 45.0;
  config.max_angle_deg = 75.0;
  config.max_rate_deg_s = 30.0;
  auto gate = seeded_gate(config);
  const auto result = gate.evaluate(
    bearing(100.0), bearing(30.0), 1.0, 30.0, 11.34, true);
  EXPECT_EQ(result.decision, BearingGateDecision::Accept);
  EXPECT_NEAR(result.allowed_deg, 75.0, 1e-9);
}

TEST(BearingContinuityGate, AllowsLargeResidualWhenPredictionIsUncertain)
{
  auto gate = seeded_gate();
  const auto result = gate.evaluate(
    bearing(90.0), bearing(30.0), 0.1, 2.0, 11.34, true);
  EXPECT_EQ(result.decision, BearingGateDecision::Accept);
}

TEST(BearingContinuityGate, RejectsBeyondHardCapEvenWhenPredictionIsUncertain)
{
  auto gate = seeded_gate();
  const auto result = gate.evaluate(
    bearing(125.0), bearing(30.0), 0.1, 2.0, 11.34, true);
  EXPECT_EQ(result.decision, BearingGateDecision::Reject);
  EXPECT_EQ(result.pending_count, 1);
}

TEST(BearingContinuityGate, ReacquiresPersistentChallenger)
{
  BearingGateConfig config;
  config.confirmations = 3;
  config.confirmation_time_s = 0.25;
  auto gate = seeded_gate(config);
  EXPECT_EQ(gate.evaluate(
    bearing(125.0), bearing(30.0), 0.1, 50.0, 11.34, true).decision,
    BearingGateDecision::Reject);
  EXPECT_EQ(gate.evaluate(
    bearing(124.0), bearing(30.0), 0.2, 50.0, 11.34, true).decision,
    BearingGateDecision::Reject);
  // Three samples alone are insufficient until the challenger also persists
  // for the configured minimum duration.
  EXPECT_EQ(gate.evaluate(
    bearing(126.0), bearing(30.0), 0.3, 50.0, 11.34, true).decision,
    BearingGateDecision::Reject);
  EXPECT_EQ(gate.evaluate(
    bearing(125.0), bearing(30.0), 0.4, 50.0, 11.34, true).decision,
    BearingGateDecision::Reacquire);
}

TEST(BearingContinuityGate, JitteringChallengersDoNotConfirm)
{
  BearingGateConfig config;
  config.reacquire_timeout_s = 10.0;
  auto gate = seeded_gate(config);
  const double candidates[] = {125.0, -70.0, 130.0, -80.0, 140.0};
  for (std::size_t index = 0; index < 5; ++index) {
    const auto result = gate.evaluate(
      bearing(candidates[index]), bearing(30.0), 0.1 * (index + 1),
      50.0, 11.34, true);
    EXPECT_EQ(result.decision, BearingGateDecision::Reject);
    EXPECT_LT(result.pending_count, config.confirmations);
  }
}

TEST(BearingContinuityGate, ReacquiresAfterTemporarySourceLoss)
{
  BearingGateConfig config;
  config.reacquire_timeout_s = 2.0;
  config.confirmations = 3;
  config.confirmation_time_s = 0.25;
  auto gate = seeded_gate(config);
  // Establish a seen candidate without accepting it, then leave a real gap.
  EXPECT_EQ(gate.evaluate(
    bearing(125.0), bearing(30.0), 0.1, 50.0, 11.34, true).decision,
    BearingGateDecision::Reject);
  const auto result = gate.evaluate(
    bearing(125.0), bearing(30.0), 2.2, 50.0, 11.34, true);
  EXPECT_EQ(result.decision, BearingGateDecision::Reject);
  EXPECT_TRUE(result.after_source_gap);
  EXPECT_EQ(gate.evaluate(
    bearing(125.0), bearing(30.0), 2.35, 50.0, 11.34, true).decision,
    BearingGateDecision::Reject);
  EXPECT_EQ(gate.evaluate(
    bearing(125.0), bearing(30.0), 2.50, 50.0, 11.34, true).decision,
    BearingGateDecision::Reacquire);
}

TEST(BearingContinuityGate, RejectedStreamDoesNotMasqueradeAsSourceLoss)
{
  BearingGateConfig config;
  config.confirmations = 20;
  auto gate = seeded_gate(config);
  for (double stamp : {0.5, 1.5, 2.5, 3.5}) {
    const auto result = gate.evaluate(
      bearing(125.0), bearing(30.0), stamp, 50.0, 11.34, true);
    EXPECT_EQ(result.decision, BearingGateDecision::Reject);
    EXPECT_FALSE(result.after_source_gap);
  }
}

TEST(BearingContinuityGate, DuplicateAndRegressedTimestampsAddNoEvidence)
{
  auto gate = seeded_gate();
  EXPECT_EQ(gate.evaluate(
    bearing(125.0), bearing(30.0), 0.1, 50.0, 11.34, true).pending_count, 1);
  EXPECT_EQ(gate.evaluate(
    bearing(125.0), bearing(30.0), 0.1, 50.0, 11.34, true).pending_count, 1);
  EXPECT_EQ(gate.evaluate(
    bearing(125.0), bearing(30.0), 0.05, 50.0, 11.34, true).pending_count, 1);
}

TEST(BearingContinuityGate, VisualAnchorDefersForcedReacquisition)
{
  BearingGateConfig config;
  config.confirmations = 3;
  config.confirmation_time_s = 0.25;
  auto gate = seeded_gate(config);
  for (double stamp : {0.1, 0.2, 0.4}) {
    const auto result = gate.evaluate(
      bearing(125.0), bearing(30.0), stamp, 50.0, 11.34, false);
    EXPECT_EQ(result.decision, BearingGateDecision::Reject);
  }
  EXPECT_EQ(gate.evaluate(
    bearing(125.0), bearing(30.0), 0.5, 50.0, 11.34, true).decision,
    BearingGateDecision::Reacquire);
}

TEST(BearingContinuityGate, DisabledGateAlwaysAccepts)
{
  BearingGateConfig config;
  config.enabled = false;
  BearingContinuityGate gate(config);
  for (double degrees : {0.0, 90.0, 180.0, -90.0}) {
    EXPECT_EQ(gate.evaluate(
      bearing(degrees), bearing(30.0), 1.0, 1000.0, 11.34, true).decision,
      BearingGateDecision::Accept);
  }
}

TEST(BearingContinuityGate, RejectsInvalidConfiguration)
{
  BearingGateConfig config;
  config.max_angle_deg = 30.0;
  EXPECT_THROW(BearingContinuityGate gate(config), std::invalid_argument);
}

}  // namespace
