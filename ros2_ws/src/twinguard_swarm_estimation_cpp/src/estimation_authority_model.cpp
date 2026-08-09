#include "twinguard_swarm_estimation_cpp/estimation_authority_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace twinguard::estimation
{

EstimationAuthorityModel::EstimationAuthorityModel(EstimationAuthorityConfig config)
: config_(config)
{
  config_.max_position_sigma_m = std::max(config_.max_position_sigma_m, 1e-6);
  config_.nis_limit = std::max(config_.nis_limit, 1e-6);
  config_.stale_timeout_ms = std::max(config_.stale_timeout_ms, 1e-6);
}

EstimationAuthorityResult EstimationAuthorityModel::evaluate(
  const EstimationAuthorityInput & input) const
{
  EstimationAuthorityResult result;
  const double max_variance = std::max({
    input.position_variance[0],
    input.position_variance[1],
    input.position_variance[2],
  });
  result.position_sigma_m = std::isfinite(max_variance) && max_variance > 0.0 ?
    std::sqrt(max_variance) : 0.0;
  result.position_nis = input.position_nis;
  result.velocity_nis = input.velocity_nis;
  result.measurement_age_ms = input.measurement_age_ms;
  result.active_sensor_mask = input.active_sensor_mask;

  result.covariance_factor = std::isfinite(result.position_sigma_m) ?
    clamp01(1.0 - result.position_sigma_m / config_.max_position_sigma_m) : 0.0;

  const double max_nis = std::max(input.position_nis, input.velocity_nis);
  result.nis_factor = std::isfinite(max_nis) ?
    clamp01(1.0 - max_nis / config_.nis_limit) : 0.0;

  result.freshness_factor = std::isfinite(input.measurement_age_ms) ?
    clamp01(1.0 - input.measurement_age_ms / config_.stale_timeout_ms) : 0.0;

  const auto required_mask = input.required_sensor_mask;
  if (required_mask == 0u) {
    result.availability_factor = 1.0;
  } else {
    const auto active_required = input.active_sensor_mask & required_mask;
    result.availability_factor =
      clamp01(static_cast<double>(popcount(active_required)) / popcount(required_mask));
  }

  result.estimation_factor = std::min({
    result.covariance_factor,
    result.nis_factor,
    result.freshness_factor,
    result.availability_factor,
  });

  result.estimation_limiting_reason = "covariance";
  double limiting_factor = result.covariance_factor;
  if (result.nis_factor < limiting_factor) {
    limiting_factor = result.nis_factor;
    result.estimation_limiting_reason = "NIS";
  }
  if (result.freshness_factor < limiting_factor) {
    limiting_factor = result.freshness_factor;
    result.estimation_limiting_reason = "freshness";
  }
  if (result.availability_factor < limiting_factor) {
    result.estimation_limiting_reason = "availability";
  }
  if (result.estimation_factor >= 1.0) {
    result.estimation_limiting_reason = "none";
  }

  return result;
}

double EstimationAuthorityModel::clamp01(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::clamp(value, 0.0, 1.0);
}

int EstimationAuthorityModel::popcount(std::uint32_t value)
{
  int count = 0;
  while (value != 0u) {
    count += static_cast<int>(value & 1u);
    value >>= 1u;
  }
  return count;
}

}  // namespace twinguard::estimation
