#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "twinguard_swarm_integrity_cpp/hard_safety_monitor.hpp"

using twinguard::integrity::HardSafetyInput;
using twinguard::integrity::HardSafetyMonitor;
using twinguard::integrity::IntegrityState;
using twinguard::integrity::OperationContext;
using twinguard::integrity::TransitionReason;

TEST(HardSafetyMonitor, NominalInputDoesNotOverride)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.state_values = {1.0, 2.0, 3.0};
  input.covariance_values = {0.1, 0.2, 0.3};

  const auto result = monitor.evaluate(input);

  EXPECT_FALSE(result.hard_override_active);
  EXPECT_DOUBLE_EQ(result.target_authority, 1.0);
  EXPECT_EQ(result.integrity_state, IntegrityState::NOMINAL);
  EXPECT_EQ(result.transition_reason, TransitionReason::NONE);
  EXPECT_EQ(result.diagnostic_reason, "none");
}

TEST(HardSafetyMonitor, StaleEstimatorForcesFloorWithinOneTick)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.authority_floor = 0.15;
  input.estimator_stale = true;
  input.state_values = {1.0, 2.0, 3.0};
  input.covariance_values = {0.1, 0.2, 0.3};

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_DOUBLE_EQ(result.target_authority, 0.15);
  EXPECT_EQ(result.integrity_state, IntegrityState::DEGRADED_HOLD);
  EXPECT_EQ(result.transition_reason, TransitionReason::HARD_SAFETY_OVERRIDE);
  EXPECT_EQ(result.diagnostic_reason, "stale_estimator");
}

TEST(HardSafetyMonitor, NanCovarianceNeverPropagates)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.covariance_required = true;
  input.covariance_values = {0.1, std::numeric_limits<double>::quiet_NaN(), 0.3};

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_TRUE(std::isfinite(result.target_authority));
  EXPECT_EQ(result.integrity_state, IntegrityState::DEGRADED_HOLD);
  EXPECT_EQ(result.diagnostic_reason, "invalid_covariance");
}

TEST(HardSafetyMonitor, NonFiniteStateNeverEntersNormalScoring)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.state_values = {1.0, std::numeric_limits<double>::infinity(), 3.0};

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_EQ(result.integrity_state, IntegrityState::DEGRADED_HOLD);
  EXPECT_EQ(result.transition_reason, TransitionReason::HARD_SAFETY_OVERRIDE);
  EXPECT_EQ(result.diagnostic_reason, "non_finite_state");
}

TEST(HardSafetyMonitor, InvalidCovarianceSetsExplicitReason)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.covariance_required = true;
  input.covariance_values = {0.1, -0.01, 0.3};

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_EQ(result.transition_reason, TransitionReason::HARD_SAFETY_OVERRIDE);
  EXPECT_EQ(result.diagnostic_reason, "invalid_covariance");
}

TEST(HardSafetyMonitor, MissingRequiredCovarianceSetsExplicitReason)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.covariance_required = true;

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_EQ(result.integrity_state, IntegrityState::DEGRADED_HOLD);
  EXPECT_EQ(result.diagnostic_reason, "invalid_covariance");
}

TEST(HardSafetyMonitor, ValidFullCovarianceAllowsNegativeOffDiagonalTerms)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.covariance_required = true;
  input.covariance_values = {
    1.0, -0.2,
    -0.2, 1.0,
  };

  const auto result = monitor.evaluate(input);

  EXPECT_FALSE(result.hard_override_active);
  EXPECT_EQ(result.integrity_state, IntegrityState::NOMINAL);
}

TEST(HardSafetyMonitor, CovarianceDecompositionFailureForcesHold)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.covariance_required = true;
  input.covariance_decomposition_failed = true;
  input.covariance_values = {
    1.0, 0.0,
    0.0, 1.0,
  };

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_DOUBLE_EQ(result.target_authority, input.authority_floor);
  EXPECT_EQ(result.integrity_state, IntegrityState::DEGRADED_HOLD);
  EXPECT_EQ(result.diagnostic_reason, "covariance_decomposition_failed");
}

TEST(HardSafetyMonitor, IndefiniteCovarianceForcesHold)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.covariance_required = true;
  input.covariance_values = {
    1.0, 2.0,
    2.0, 1.0,
  };

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_EQ(result.diagnostic_reason, "invalid_covariance");
}

TEST(HardSafetyMonitor, LossOfAllLocalizationEntersHoldImmediately)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.has_valid_localization_source = false;
  input.state_values = {0.0, 0.0, 0.0};
  input.covariance_values = {1.0, 1.0, 1.0};

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_EQ(result.integrity_state, IntegrityState::DEGRADED_HOLD);
  EXPECT_EQ(result.diagnostic_reason, "no_valid_localization_source");
}

TEST(HardSafetyMonitor, CriticalBatteryBypassesNormalAuthority)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.authority_floor = 0.2;
  input.critical_battery = true;

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_DOUBLE_EQ(result.target_authority, 0.2);
  EXPECT_EQ(result.diagnostic_reason, "critical_battery");
}

TEST(HardSafetyMonitor, ImminentCollisionBypassesDwellAndPersistence)
{
  HardSafetyMonitor monitor;
  HardSafetyInput input;
  input.authority_floor = 0.15;
  input.imminent_collision = true;
  input.state_values = {1.0, 0.0, -2.0};

  const auto result = monitor.evaluate(input);

  EXPECT_TRUE(result.hard_override_active);
  EXPECT_DOUBLE_EQ(result.target_authority, 0.15);
  EXPECT_EQ(result.integrity_state, IntegrityState::DEGRADED_HOLD);
  EXPECT_EQ(result.operation_context, OperationContext::STEADY_STATE_DEGRADED);
  EXPECT_EQ(result.transition_reason, TransitionReason::HARD_SAFETY_OVERRIDE);
  EXPECT_EQ(result.diagnostic_reason, "imminent_collision");
}
