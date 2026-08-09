#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace twinguard::integrity
{

enum class IntegrityState
{
  NOMINAL,
  LIMITED_OPERATION,
  DEGRADED_HOLD,
};

enum class OperationContext
{
  NORMAL,
  STEADY_STATE_DEGRADED,
  RECOVERING_FROM_HOLD,
};

enum class TransitionReason
{
  NONE,
  HARD_SAFETY_OVERRIDE,
};

struct HardSafetyInput
{
  double authority_floor{0.15};
  bool estimator_stale{false};
  bool has_valid_localization_source{true};
  bool covariance_decomposition_failed{false};
  bool critical_battery{false};
  bool imminent_collision{false};
  bool covariance_required{false};
  std::vector<double> state_values{};
  std::vector<double> covariance_values{};
};

struct HardSafetyResult
{
  bool hard_override_active{false};
  double target_authority{1.0};
  IntegrityState integrity_state{IntegrityState::NOMINAL};
  OperationContext operation_context{OperationContext::NORMAL};
  TransitionReason transition_reason{TransitionReason::NONE};
  std::string diagnostic_reason{"none"};
};

class HardSafetyMonitor
{
public:
  HardSafetyResult evaluate(const HardSafetyInput & input) const;

private:
  static bool all_finite(const std::vector<double> & values);
  static bool covariance_valid(const std::vector<double> & values, bool required);
  static bool covariance_matrix_valid(const std::vector<double> & values, std::size_t dimension);
  static HardSafetyResult override_result(double authority_floor, const std::string & reason);
};

const char * to_string(IntegrityState state);
const char * to_string(OperationContext context);
const char * to_string(TransitionReason reason);

}  // namespace twinguard::integrity
