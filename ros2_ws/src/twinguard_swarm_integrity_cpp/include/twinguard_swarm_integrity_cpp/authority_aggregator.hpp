#pragma once

#include <string>

namespace twinguard::integrity
{

struct AuthorityFactors
{
  double estimation_factor{1.0};
  double communication_factor{1.0};
  double battery_factor{1.0};
  double proximity_factor{1.0};
};

struct AuthorityDynamicsConfig
{
  double fast_fall_rate_per_s{4.0};
  double slow_rise_rate_per_s{0.5};
};

struct AuthorityAggregationResult
{
  double target_authority{1.0};
  double applied_authority{1.0};
  std::string active_limiting_factor{"none"};
};

class AuthorityAggregator
{
public:
  explicit AuthorityAggregator(AuthorityDynamicsConfig config = {});

  AuthorityAggregationResult update(
    const AuthorityFactors & factors,
    double previous_applied_authority,
    double dt_s) const;

  AuthorityAggregationResult evaluate_target(
    const AuthorityFactors & factors,
    double applied_authority) const;

private:
  static double clamp01(double value);
  static double sanitize_factor(double value, bool & valid);
  static double slew_limit(
    double previous,
    double target,
    double dt_s,
    const AuthorityDynamicsConfig & config);

  AuthorityDynamicsConfig config_;
};

}  // namespace twinguard::integrity
