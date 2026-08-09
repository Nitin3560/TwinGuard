#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "twinguard_swarm_integrity_cpp/authority_aggregator.hpp"

using twinguard::integrity::AuthorityAggregator;
using twinguard::integrity::AuthorityDynamicsConfig;
using twinguard::integrity::AuthorityFactors;

TEST(AuthorityAggregator, TargetAuthorityEqualsMinimumFactor)
{
  AuthorityAggregator aggregator;
  AuthorityFactors factors;
  factors.estimation_factor = 0.8;
  factors.communication_factor = 0.6;
  factors.battery_factor = 0.9;
  factors.proximity_factor = 0.7;

  const auto result = aggregator.evaluate_target(factors, 1.0);

  EXPECT_DOUBLE_EQ(result.target_authority, 0.6);
  EXPECT_EQ(result.active_limiting_factor, "communication");
}

TEST(AuthorityAggregator, LimitingFactorTieBreakIsDeterministic)
{
  AuthorityAggregator aggregator;
  AuthorityFactors factors;
  factors.estimation_factor = 0.5;
  factors.communication_factor = 0.5;
  factors.battery_factor = 0.5;
  factors.proximity_factor = 0.5;

  const auto result = aggregator.evaluate_target(factors, 1.0);

  EXPECT_DOUBLE_EQ(result.target_authority, 0.5);
  EXPECT_EQ(result.active_limiting_factor, "estimation");
}

TEST(AuthorityAggregator, InvalidFactorFailsSafe)
{
  AuthorityAggregator aggregator;
  AuthorityFactors factors;
  factors.estimation_factor = 0.9;
  factors.communication_factor = std::numeric_limits<double>::quiet_NaN();
  factors.battery_factor = 0.9;
  factors.proximity_factor = 0.9;

  const auto result = aggregator.update(factors, 1.0, 0.1);

  EXPECT_DOUBLE_EQ(result.target_authority, 0.0);
  EXPECT_EQ(result.active_limiting_factor, "invalid_factor");
  EXPECT_LT(result.applied_authority, 1.0);
}

TEST(AuthorityAggregator, FallingAuthorityRespectsFallRate)
{
  AuthorityDynamicsConfig config;
  config.fast_fall_rate_per_s = 2.0;
  config.slow_rise_rate_per_s = 0.25;
  AuthorityAggregator aggregator(config);
  AuthorityFactors factors;
  factors.estimation_factor = 0.2;

  const auto result = aggregator.update(factors, 1.0, 0.1);

  EXPECT_DOUBLE_EQ(result.target_authority, 0.2);
  EXPECT_DOUBLE_EQ(result.applied_authority, 0.8);
}

TEST(AuthorityAggregator, RisingAuthorityRespectsRecoveryRate)
{
  AuthorityDynamicsConfig config;
  config.fast_fall_rate_per_s = 2.0;
  config.slow_rise_rate_per_s = 0.25;
  AuthorityAggregator aggregator(config);
  AuthorityFactors factors;
  factors.estimation_factor = 1.0;

  const auto result = aggregator.update(factors, 0.2, 0.4);

  EXPECT_DOUBLE_EQ(result.target_authority, 1.0);
  EXPECT_DOUBLE_EQ(result.applied_authority, 0.3);
}

TEST(AuthorityAggregator, IdenticalReplayProducesIdenticalAuthorityTrace)
{
  AuthorityDynamicsConfig config;
  config.fast_fall_rate_per_s = 3.0;
  config.slow_rise_rate_per_s = 0.5;
  AuthorityAggregator aggregator(config);
  std::vector<AuthorityFactors> replay;
  replay.push_back({1.0, 1.0, 1.0, 1.0});
  replay.push_back({0.7, 1.0, 1.0, 1.0});
  replay.push_back({0.4, 0.9, 1.0, 1.0});
  replay.push_back({0.9, 1.0, 1.0, 1.0});
  replay.push_back({1.0, 1.0, 1.0, 1.0});

  auto run_replay = [&]() {
    std::vector<double> trace;
    double applied = 1.0;
    for (const auto & factors : replay) {
      const auto result = aggregator.update(factors, applied, 0.1);
      applied = result.applied_authority;
      trace.push_back(applied);
    }
    return trace;
  };

  EXPECT_EQ(run_replay(), run_replay());
}
