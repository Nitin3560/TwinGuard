#include <gtest/gtest.h>

#include <cmath>

#include "twinguard_swarm_estimation_cpp/estimation_authority_model.hpp"

using twinguard::estimation::EstimationAuthorityInput;
using twinguard::estimation::EstimationAuthorityModel;
using twinguard::estimation::SENSOR_PX4_ODOMETRY;
using twinguard::estimation::SENSOR_VISUAL_ODOMETRY;

TEST(EstimationAuthorityModel, IncreasingCovarianceNeverIncreasesAuthority)
{
  EstimationAuthorityModel model;
  EstimationAuthorityInput low_covariance;
  low_covariance.position_variance = {0.2 * 0.2, 0.2 * 0.2, 0.2 * 0.2};
  low_covariance.active_sensor_mask = SENSOR_PX4_ODOMETRY;

  EstimationAuthorityInput high_covariance = low_covariance;
  high_covariance.position_variance = {0.7 * 0.7, 0.7 * 0.7, 0.7 * 0.7};

  const auto low_result = model.evaluate(low_covariance);
  const auto high_result = model.evaluate(high_covariance);

  EXPECT_LE(high_result.covariance_factor, low_result.covariance_factor);
  EXPECT_LE(high_result.estimation_factor, low_result.estimation_factor);
}

TEST(EstimationAuthorityModel, IncreasingNisNeverIncreasesAuthority)
{
  EstimationAuthorityModel model;
  EstimationAuthorityInput low_nis;
  low_nis.position_variance = {0.01, 0.01, 0.01};
  low_nis.position_nis = 1.0;
  low_nis.active_sensor_mask = SENSOR_PX4_ODOMETRY;

  EstimationAuthorityInput high_nis = low_nis;
  high_nis.position_nis = 8.0;

  const auto low_result = model.evaluate(low_nis);
  const auto high_result = model.evaluate(high_nis);

  EXPECT_LE(high_result.nis_factor, low_result.nis_factor);
  EXPECT_LE(high_result.estimation_factor, low_result.estimation_factor);
}

TEST(EstimationAuthorityModel, StaleDataMonotonicallyReducesFreshness)
{
  EstimationAuthorityModel model;
  EstimationAuthorityInput fresh;
  fresh.position_variance = {0.01, 0.01, 0.01};
  fresh.measurement_age_ms = 50.0;
  fresh.active_sensor_mask = SENSOR_PX4_ODOMETRY;

  EstimationAuthorityInput stale = fresh;
  stale.measurement_age_ms = 400.0;

  const auto fresh_result = model.evaluate(fresh);
  const auto stale_result = model.evaluate(stale);

  EXPECT_LE(stale_result.freshness_factor, fresh_result.freshness_factor);
  EXPECT_LE(stale_result.estimation_factor, fresh_result.estimation_factor);
}

TEST(EstimationAuthorityModel, MissingSensorsReduceAvailability)
{
  EstimationAuthorityModel model;
  EstimationAuthorityInput complete;
  complete.position_variance = {0.01, 0.01, 0.01};
  complete.required_sensor_mask = SENSOR_PX4_ODOMETRY | SENSOR_VISUAL_ODOMETRY;
  complete.active_sensor_mask = SENSOR_PX4_ODOMETRY | SENSOR_VISUAL_ODOMETRY;

  EstimationAuthorityInput missing = complete;
  missing.active_sensor_mask = SENSOR_PX4_ODOMETRY;

  const auto complete_result = model.evaluate(complete);
  const auto missing_result = model.evaluate(missing);

  EXPECT_DOUBLE_EQ(complete_result.availability_factor, 1.0);
  EXPECT_DOUBLE_EQ(missing_result.availability_factor, 0.5);
  EXPECT_LE(missing_result.estimation_factor, complete_result.estimation_factor);
}

TEST(EstimationAuthorityModel, ExampleFixtureLimitsOnNis)
{
  EstimationAuthorityModel model;
  EstimationAuthorityInput input;
  input.position_variance = {0.18 * 0.18, 0.18 * 0.18, 0.18 * 0.18};
  input.position_nis = 6.9;
  input.velocity_nis = 1.0;
  input.measurement_age_ms = 30.0;
  input.active_sensor_mask = SENSOR_PX4_ODOMETRY;

  const auto result = model.evaluate(input);

  EXPECT_NEAR(result.covariance_factor, 0.82, 1e-12);
  EXPECT_NEAR(result.nis_factor, 0.31, 1e-12);
  EXPECT_NEAR(result.freshness_factor, 0.94, 1e-12);
  EXPECT_NEAR(result.availability_factor, 1.00, 1e-12);
  EXPECT_NEAR(result.estimation_factor, 0.31, 1e-12);
  EXPECT_EQ(result.estimation_limiting_reason, "NIS");
}
