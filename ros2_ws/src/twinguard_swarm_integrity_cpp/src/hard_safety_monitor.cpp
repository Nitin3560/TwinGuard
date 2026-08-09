#include "twinguard_swarm_integrity_cpp/hard_safety_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace twinguard::integrity
{

HardSafetyResult HardSafetyMonitor::evaluate(const HardSafetyInput & input) const
{
  if (!all_finite(input.state_values)) {
    return override_result(input.authority_floor, "non_finite_state");
  }

  if (!covariance_valid(input.covariance_values, input.covariance_required)) {
    return override_result(input.authority_floor, "invalid_covariance");
  }

  if (input.covariance_decomposition_failed) {
    return override_result(input.authority_floor, "covariance_decomposition_failed");
  }

  if (input.estimator_stale) {
    return override_result(input.authority_floor, "stale_estimator");
  }

  if (!input.has_valid_localization_source) {
    return override_result(input.authority_floor, "no_valid_localization_source");
  }

  if (input.critical_battery) {
    return override_result(input.authority_floor, "critical_battery");
  }

  if (input.imminent_collision) {
    return override_result(input.authority_floor, "imminent_collision");
  }

  return HardSafetyResult{};
}

bool HardSafetyMonitor::all_finite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

bool HardSafetyMonitor::covariance_valid(const std::vector<double> & values, bool required)
{
  if (values.empty()) {
    return !required;
  }

  if (!all_finite(values)) {
    return false;
  }

  const auto root = static_cast<std::size_t>(std::sqrt(static_cast<double>(values.size())));
  if (root * root == values.size() && root > 1) {
    return covariance_matrix_valid(values, root);
  }

  return std::all_of(values.begin(), values.end(), [](double value) {
    return value >= 0.0;
  });
}

bool HardSafetyMonitor::covariance_matrix_valid(
  const std::vector<double> & values,
  std::size_t dimension)
{
  constexpr double kSymmetryTolerance = 1e-9;
  constexpr double kPositiveTolerance = 1e-9;
  std::vector<double> chol(values.size(), 0.0);

  for (std::size_t row = 0; row < dimension; ++row) {
    if (values[row * dimension + row] < -kPositiveTolerance) {
      return false;
    }

    for (std::size_t col = row + 1; col < dimension; ++col) {
      const double a = values[row * dimension + col];
      const double b = values[col * dimension + row];
      if (std::abs(a - b) > kSymmetryTolerance) {
        return false;
      }
    }
  }

  for (std::size_t i = 0; i < dimension; ++i) {
    for (std::size_t j = 0; j <= i; ++j) {
      double sum = values[i * dimension + j];
      for (std::size_t k = 0; k < j; ++k) {
        sum -= chol[i * dimension + k] * chol[j * dimension + k];
      }

      if (i == j) {
        if (sum < -kPositiveTolerance) {
          return false;
        }
        chol[i * dimension + j] = std::sqrt(std::max(sum, 0.0));
      } else if (chol[j * dimension + j] > kPositiveTolerance) {
        chol[i * dimension + j] = sum / chol[j * dimension + j];
      } else if (std::abs(sum) > kPositiveTolerance) {
        return false;
      }
    }
  }

  return true;
}

HardSafetyResult HardSafetyMonitor::override_result(
  double authority_floor,
  const std::string & reason)
{
  HardSafetyResult result;
  result.hard_override_active = true;
  result.target_authority = std::clamp(authority_floor, 0.0, 1.0);
  result.integrity_state = IntegrityState::DEGRADED_HOLD;
  result.operation_context = OperationContext::STEADY_STATE_DEGRADED;
  result.transition_reason = TransitionReason::HARD_SAFETY_OVERRIDE;
  result.diagnostic_reason = reason;
  return result;
}

const char * to_string(IntegrityState state)
{
  switch (state) {
    case IntegrityState::NOMINAL:
      return "nominal";
    case IntegrityState::LIMITED_OPERATION:
      return "limited_operation";
    case IntegrityState::DEGRADED_HOLD:
      return "degraded_hold";
  }
  return "unknown";
}

const char * to_string(OperationContext context)
{
  switch (context) {
    case OperationContext::NORMAL:
      return "normal";
    case OperationContext::STEADY_STATE_DEGRADED:
      return "steady_state_degraded";
    case OperationContext::RECOVERING_FROM_HOLD:
      return "recovering_from_hold";
  }
  return "unknown";
}

const char * to_string(TransitionReason reason)
{
  switch (reason) {
    case TransitionReason::NONE:
      return "none";
    case TransitionReason::HARD_SAFETY_OVERRIDE:
      return "hard_safety_override";
  }
  return "unknown";
}

}  // namespace twinguard::integrity
