#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "twinguard_swarm_estimation_cpp/ekf_estimator.hpp"

using twinguard::estimation::EkfEstimator;

TEST(EkfEstimator, PredictPropagatesPositionAndCovariance)
{
  EkfEstimator ekf(0.5);
  ekf.reset({1.0, 2.0, 3.0}, {0.5, -1.0, 2.0});
  const double initial_position_var = ekf.state().P(0, 0);

  ekf.predict(2.0);
  const auto & state = ekf.state();

  EXPECT_NEAR(state.x(0), 2.0, 1e-9);
  EXPECT_NEAR(state.x(1), 0.0, 1e-9);
  EXPECT_NEAR(state.x(2), 7.0, 1e-9);
  EXPECT_GT(state.P(0, 0), initial_position_var);
  EXPECT_GT(state.P(3, 3), 1.0);
}

TEST(EkfEstimator, PositionUpdateShrinksCovarianceAndReturnsNis)
{
  EkfEstimator ekf(0.5);
  ekf.reset({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  ekf.predict(1.0);

  const double before_var = ekf.state().P(0, 0);
  const double nis = ekf.update_position({1.5, 0.0, 0.0}, 0.5);
  const auto & state = ekf.state();

  EXPECT_NEAR(nis, 0.1, 1e-9);
  EXPECT_NEAR(state.nis, nis, 1e-12);
  EXPECT_NEAR(state.x(0), 1.45, 1e-9);
  EXPECT_GT(state.x(3), 1.0);
  EXPECT_LT(state.P(0, 0), before_var);
}

TEST(EkfEstimator, VelocityUpdateCorrectsVelocityState)
{
  EkfEstimator ekf(0.2);
  ekf.reset({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});

  const double nis = ekf.update_velocity({2.0, 0.0, 0.0}, 1.0);
  const auto & state = ekf.state();

  EXPECT_NEAR(nis, 2.0, 1e-9);
  EXPECT_NEAR(state.x(3), 1.0, 1e-9);
  EXPECT_NEAR(state.x(4), 0.0, 1e-9);
  EXPECT_NEAR(state.x(5), 0.0, 1e-9);
}

TEST(EkfEstimator, NearZeroMeasurementNoiseRemainsFinite)
{
  EkfEstimator ekf(0.0);
  ekf.reset({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});

  const double nis = ekf.update_position({0.1, 0.0, 0.0}, 0.0);
  const auto & state = ekf.state();

  EXPECT_TRUE(std::isfinite(nis));
  EXPECT_TRUE(state.x.allFinite());
  EXPECT_TRUE(state.P.allFinite());
  EXPECT_NEAR(state.x(0), 0.1, 1e-9);
}
