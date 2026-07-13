#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "twinguard_swarm_integrity_cpp/trust_scorer.hpp"

using twinguard::integrity::DigitalTwinPredictor;
using twinguard::integrity::TrustScorer;

TEST(TrustScorer, InitialStateIsNominal)
{
  TrustScorer scorer;
  const auto & state = scorer.state();
  EXPECT_DOUBLE_EQ(state.trust, 1.0);
  EXPECT_DOUBLE_EQ(state.residual, 0.0);
  EXPECT_DOUBLE_EQ(state.authority_scale, 1.0);
  EXPECT_EQ(state.fault_label, "nominal");
}

TEST(TrustScorer, ZeroResidualKeepsFullTrust)
{
  TrustScorer scorer;
  const std::array<double, 3> pos{1.0, 2.0, 3.0};
  for (int i = 0; i < 5; ++i) {
    const auto result = scorer.update(pos, pos);
    EXPECT_NEAR(result.residual, 0.0, 1e-9);
    EXPECT_NEAR(result.trust, 1.0, 1e-9);
    EXPECT_NEAR(result.authority_scale, 1.0, 1e-9);
    EXPECT_EQ(result.fault_label, "nominal");
  }
}

TEST(TrustScorer, SingleUpdateMatchesExpectedFormula)
{
  TrustScorer scorer;
  const std::array<double, 3> measured{1.0, 0.0, 0.0};
  const std::array<double, 3> predicted{0.0, 0.0, 0.0};

  const double expected_residual = 1.0;
  const double expected_integrity = std::exp(-1.2 * expected_residual);
  const double expected_trust = 0.90 * 1.0 + 0.10 * expected_integrity;
  const auto result = scorer.update(measured, predicted);

  EXPECT_NEAR(result.residual, expected_residual, 1e-9);
  EXPECT_NEAR(result.trust, expected_trust, 1e-9);
  EXPECT_NEAR(result.authority_scale, expected_trust, 1e-9);
  EXPECT_EQ(result.fault_label, "nominal");
}

TEST(TrustScorer, SustainedLargeResidualDegradesThenTriggersAttackLabel)
{
  TrustScorer scorer;
  const std::array<double, 3> predicted{0.0, 0.0, 0.0};
  const std::array<double, 3> measured{5.0, 0.0, 0.0};

  twinguard::integrity::TrustState last{};
  for (int i = 0; i < 7; ++i) {
    last = scorer.update(measured, predicted);
  }
  EXPECT_LT(last.trust, 0.65);
  EXPECT_GE(last.trust, 0.35);
  EXPECT_EQ(last.fault_label, "degraded");

  for (int i = 0; i < 30; ++i) {
    last = scorer.update(measured, predicted);
  }
  EXPECT_LT(last.trust, 0.35);
  EXPECT_GT(last.residual, 1.0);
  EXPECT_EQ(last.fault_label, "suspected_attack");
}

TEST(TrustScorer, AuthorityNeverDropsBelowMinimum)
{
  constexpr double min_authority = 0.15;
  TrustScorer scorer(1.2, 0.90, min_authority);
  const std::array<double, 3> predicted{0.0, 0.0, 0.0};
  const std::array<double, 3> measured{50.0, 0.0, 0.0};

  twinguard::integrity::TrustState last{};
  for (int i = 0; i < 100; ++i) {
    last = scorer.update(measured, predicted);
  }
  EXPECT_GE(last.authority_scale, min_authority);
  EXPECT_NEAR(last.authority_scale, min_authority, 1e-6);
}

TEST(DigitalTwinPredictor, PredictWithoutResetStaysAtOrigin)
{
  DigitalTwinPredictor predictor;
  for (int i = 0; i < 3; ++i) {
    const auto pos = predictor.predict();
    EXPECT_DOUBLE_EQ(pos[0], 0.0);
    EXPECT_DOUBLE_EQ(pos[1], 0.0);
    EXPECT_DOUBLE_EQ(pos[2], 0.0);
  }
}

TEST(DigitalTwinPredictor, PredictIntegratesPositionByVelocity)
{
  DigitalTwinPredictor predictor(0.02);
  predictor.reset({1.0, 2.0, 3.0}, {1.0, 0.0, -1.0});

  const auto first = predictor.predict();
  EXPECT_NEAR(first[0], 1.02, 1e-9);
  EXPECT_NEAR(first[1], 2.0, 1e-9);
  EXPECT_NEAR(first[2], 2.98, 1e-9);

  const auto second = predictor.predict();
  EXPECT_NEAR(second[0], 1.04, 1e-9);
  EXPECT_NEAR(second[1], 2.0, 1e-9);
  EXPECT_NEAR(second[2], 2.96, 1e-9);
}

TEST(DigitalTwinPredictor, CorrectVelocityActsAsExponentialMovingAverage)
{
  DigitalTwinPredictor predictor(1.0);
  predictor.reset({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});

  predictor.correct_velocity({10.0, 0.0, 0.0}, 0.2);
  const auto pos = predictor.predict();
  EXPECT_NEAR(pos[0], 2.0, 1e-9);
  EXPECT_NEAR(pos[1], 0.0, 1e-9);
  EXPECT_NEAR(pos[2], 0.0, 1e-9);
}
