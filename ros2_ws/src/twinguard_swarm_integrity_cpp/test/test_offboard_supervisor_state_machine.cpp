#include <gtest/gtest.h>

#include <array>

#include "twinguard_swarm_integrity_cpp/offboard_supervisor.hpp"

using twinguard::offboard::OffboardSupervisor;
using twinguard::offboard::OperationContext;
using twinguard::offboard::SupervisorMode;
using twinguard::offboard::SupervisorThresholds;

namespace
{

constexpr std::array<double, 3> kCurrent{1.0, 2.0, -2.0};
constexpr std::array<double, 3> kNominal{5.0, 2.0, -2.0};

SupervisorThresholds fast_thresholds()
{
  SupervisorThresholds thresholds;
  thresholds.nominal_exit_threshold = 0.75;
  thresholds.nominal_enter_threshold = 0.85;
  thresholds.hold_threshold = 0.25;
  thresholds.hold_exit_threshold = 0.45;
  thresholds.transition_dwell_s = 0.5;
  return thresholds;
}

}  // namespace

TEST(OffboardSupervisorStateMachine, NoisyThresholdsDoNotChatter)
{
  OffboardSupervisor supervisor(3.0, 0.5, fast_thresholds());

  for (int i = 0; i < 12; ++i) {
    const double target = (i % 2 == 0) ? 0.74 : 0.76;
    supervisor.step(target, target, "nominal", kCurrent, kNominal, 0.0, 0.1);
    EXPECT_EQ(supervisor.mode(), SupervisorMode::NOMINAL);
    EXPECT_EQ(supervisor.operation_context(), OperationContext::NORMAL);
  }
}

TEST(OffboardSupervisorStateMachine, HardFaultBypassesDwell)
{
  OffboardSupervisor supervisor(3.0, 0.5, fast_thresholds());

  const auto command = supervisor.step(
    1.0, 1.0, "hard_safety_override", kCurrent, kNominal, 0.0, 0.0);

  EXPECT_EQ(supervisor.mode(), SupervisorMode::DEGRADED_HOLD);
  EXPECT_TRUE(command.hold);
  EXPECT_DOUBLE_EQ(command.velocity_limit, 0.0);
  EXPECT_EQ(command.position, kCurrent);
}

TEST(OffboardSupervisorStateMachine, RecoveryCannotHappenBeforeDwell)
{
  OffboardSupervisor supervisor(3.0, 0.5, fast_thresholds());
  supervisor.step(1.0, 1.0, "hard_safety_override", kCurrent, kNominal, 0.0, 0.0);

  const auto command = supervisor.step(0.8, 0.8, "nominal", kCurrent, kNominal, 0.0, 0.2);

  EXPECT_EQ(supervisor.mode(), SupervisorMode::DEGRADED_HOLD);
  EXPECT_TRUE(command.hold);
  EXPECT_EQ(command.position, kCurrent);
}

TEST(OffboardSupervisorStateMachine, RecoveryContextClearsOnlyAfterNominalTransition)
{
  OffboardSupervisor supervisor(3.0, 0.5, fast_thresholds());
  supervisor.step(1.0, 1.0, "hard_safety_override", kCurrent, kNominal, 0.0, 0.0);

  supervisor.step(0.6, 0.6, "nominal", kCurrent, kNominal, 0.0, 0.5);
  EXPECT_EQ(supervisor.mode(), SupervisorMode::LIMITED_OPERATION);
  EXPECT_EQ(supervisor.operation_context(), OperationContext::RECOVERING_FROM_HOLD);

  supervisor.step(0.6, 0.6, "nominal", kCurrent, kNominal, 0.0, 1.0);
  EXPECT_EQ(supervisor.mode(), SupervisorMode::LIMITED_OPERATION);
  EXPECT_EQ(supervisor.operation_context(), OperationContext::RECOVERING_FROM_HOLD);

  supervisor.step(0.9, 0.9, "nominal", kCurrent, kNominal, 0.0, 0.5);
  EXPECT_EQ(supervisor.mode(), SupervisorMode::NOMINAL);
  EXPECT_EQ(supervisor.operation_context(), OperationContext::NORMAL);
}

TEST(OffboardSupervisorStateMachine, SameAuthorityCanBeSteadyOrRecovering)
{
  OffboardSupervisor steady(3.0, 0.5, fast_thresholds());
  steady.step(0.6, 0.6, "nominal", kCurrent, kNominal, 0.0, 0.5);

  OffboardSupervisor recovering(3.0, 0.5, fast_thresholds());
  recovering.step(1.0, 1.0, "hard_safety_override", kCurrent, kNominal, 0.0, 0.0);
  recovering.step(0.6, 0.6, "nominal", kCurrent, kNominal, 0.0, 0.5);

  EXPECT_EQ(steady.mode(), SupervisorMode::LIMITED_OPERATION);
  EXPECT_EQ(recovering.mode(), SupervisorMode::LIMITED_OPERATION);
  EXPECT_DOUBLE_EQ(steady.commanded_authority(), recovering.commanded_authority());
  EXPECT_EQ(steady.operation_context(), OperationContext::STEADY_STATE_DEGRADED);
  EXPECT_EQ(recovering.operation_context(), OperationContext::RECOVERING_FROM_HOLD);
}

TEST(OffboardSupervisorStateMachine, TrustStateCompatibilityOverloadStillWorks)
{
  OffboardSupervisor supervisor;

  const auto command = supervisor.step(1.0, "nominal", kCurrent, kNominal, 0.25);

  EXPECT_EQ(supervisor.mode(), SupervisorMode::NOMINAL);
  EXPECT_EQ(supervisor.operation_context(), OperationContext::NORMAL);
  EXPECT_FALSE(command.hold);
  EXPECT_DOUBLE_EQ(command.velocity_limit, 3.0);
  EXPECT_EQ(command.position, kNominal);
}
