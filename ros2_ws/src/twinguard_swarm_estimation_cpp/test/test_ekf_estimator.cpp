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

TEST(EkfEstimator, NegativeDtUsesMinimumStep)
{
  EkfEstimator ekf(0.5);
  ekf.reset({1.0, 2.0, 3.0}, {10.0, 0.0, -10.0});

  ekf.predict(-1.0);
  const auto & state = ekf.state();

  EXPECT_NEAR(state.x(0), 1.001, 1e-9);
  EXPECT_NEAR(state.x(1), 2.0, 1e-9);
  EXPECT_NEAR(state.x(2), 2.999, 1e-9);
}

TEST(EkfEstimator, RepeatedPredictUpdateRemainsFiniteAndSymmetric)
{
  EkfEstimator ekf(0.3);
  ekf.reset({0.0, 0.0, 0.0}, {1.0, 0.1, -0.1});

  for (int i = 0; i < 200; ++i) {
    ekf.predict(0.05);
    const double t = static_cast<double>(i + 1) * 0.05;
    ekf.update_position({t, 0.1 * t, -0.1 * t}, 0.2);
    ekf.update_velocity({1.0, 0.1, -0.1}, 0.3);
  }

  const auto & state = ekf.state();
  EXPECT_TRUE(state.x.allFinite());
  EXPECT_TRUE(state.P.allFinite());
  EXPECT_NEAR((state.P - state.P.transpose()).norm(), 0.0, 1e-9);
  EXPECT_GE(state.P.diagonal().minCoeff(), -1e-9);
  EXPECT_LT((state.x.head<3>() - Eigen::Vector3d(10.0, 1.0, -1.0)).norm(), 0.1);
}

TEST(EkfEstimator, SequentialPositionAndVelocityUpdatesCorrectBothSubstates)
{
  EkfEstimator ekf(0.5);
  ekf.reset({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});

  ekf.update_position({2.0, 0.0, 0.0}, 0.5);
  ekf.update_velocity({1.0, 0.0, 0.0}, 0.5);
  const auto & state = ekf.state();

  EXPECT_GT(state.x(0), 1.0);
  EXPECT_GT(state.x(3), 0.5);
  EXPECT_NEAR(state.x(1), 0.0, 1e-9);
  EXPECT_NEAR(state.x(4), 0.0, 1e-9);
}

TEST(EkfEstimator, ResetRestoresStateAndCovariance)
{
  EkfEstimator ekf(0.5);
  ekf.reset({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});
  ekf.predict(1.0);
  ekf.update_position({10.0, 10.0, 10.0}, 0.5);

  ekf.reset({-1.0, -2.0, -3.0}, {0.1, 0.2, 0.3});
  const auto & state = ekf.state();

  EXPECT_NEAR(state.x(0), -1.0, 1e-12);
  EXPECT_NEAR(state.x(1), -2.0, 1e-12);
  EXPECT_NEAR(state.x(2), -3.0, 1e-12);
  EXPECT_NEAR(state.x(3), 0.1, 1e-12);
  EXPECT_NEAR(state.x(4), 0.2, 1e-12);
  EXPECT_NEAR(state.x(5), 0.3, 1e-12);
  EXPECT_NEAR((state.P - Eigen::Matrix<double, 6, 6>::Identity()).norm(), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(state.nis, 0.0);
}
