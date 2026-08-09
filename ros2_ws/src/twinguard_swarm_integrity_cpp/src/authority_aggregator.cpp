#include "twinguard_swarm_integrity_cpp/authority_aggregator.hpp"

#include <algorithm>
#include <cmath>

namespace twinguard::integrity
{

AuthorityAggregator::AuthorityAggregator(AuthorityDynamicsConfig config)
: config_(config)
{
  config_.fast_fall_rate_per_s = std::max(config_.fast_fall_rate_per_s, 0.0);
  config_.slow_rise_rate_per_s = std::max(config_.slow_rise_rate_per_s, 0.0);
}

AuthorityAggregationResult AuthorityAggregator::update(
  const AuthorityFactors & factors,
  double previous_applied_authority,
  double dt_s) const
{
  auto result = evaluate_target(factors, previous_applied_authority);
  result.applied_authority = slew_limit(
    previous_applied_authority,
    result.target_authority,
    dt_s,
    config_);
  return result;
}

AuthorityAggregationResult AuthorityAggregator::evaluate_target(
  const AuthorityFactors & factors,
  double applied_authority) const
{
  AuthorityAggregationResult result;
  result.applied_authority = clamp01(applied_authority);

  bool valid = true;
  const double estimation = sanitize_factor(factors.estimation_factor, valid);
  if (!valid) {
    result.target_authority = 0.0;
    result.active_limiting_factor = "invalid_factor";
    return result;
  }

  const double communication = sanitize_factor(factors.communication_factor, valid);
  if (!valid) {
    result.target_authority = 0.0;
    result.active_limiting_factor = "invalid_factor";
    return result;
  }

  const double battery = sanitize_factor(factors.battery_factor, valid);
  if (!valid) {
    result.target_authority = 0.0;
    result.active_limiting_factor = "invalid_factor";
    return result;
  }

  const double proximity = sanitize_factor(factors.proximity_factor, valid);
  if (!valid) {
    result.target_authority = 0.0;
    result.active_limiting_factor = "invalid_factor";
    return result;
  }

  result.target_authority = estimation;
  result.active_limiting_factor = "estimation";
  if (communication < result.target_authority) {
    result.target_authority = communication;
    result.active_limiting_factor = "communication";
  }
  if (battery < result.target_authority) {
    result.target_authority = battery;
    result.active_limiting_factor = "battery";
  }
  if (proximity < result.target_authority) {
    result.target_authority = proximity;
    result.active_limiting_factor = "proximity";
  }
  if (result.target_authority >= 1.0) {
    result.active_limiting_factor = "none";
  }

  return result;
}

double AuthorityAggregator::clamp01(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::clamp(value, 0.0, 1.0);
}

double AuthorityAggregator::sanitize_factor(double value, bool & valid)
{
  if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
    valid = false;
    return 0.0;
  }
  return value;
}

double AuthorityAggregator::slew_limit(
  double previous,
  double target,
  double dt_s,
  const AuthorityDynamicsConfig & config)
{
  const double prior = clamp01(previous);
  const double desired = clamp01(target);
  const double dt = std::max(dt_s, 0.0);
  if (desired < prior) {
    return std::max(desired, prior - config.fast_fall_rate_per_s * dt);
  }
  return std::min(desired, prior + config.slow_rise_rate_per_s * dt);
}

}  // namespace twinguard::integrity
