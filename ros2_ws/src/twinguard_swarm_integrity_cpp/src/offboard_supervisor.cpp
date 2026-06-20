#include "twinguard_swarm_integrity_cpp/offboard_supervisor.hpp"

#include <algorithm>
#include <cstddef>

namespace twinguard::offboard
{

OffboardSupervisor::OffboardSupervisor(double nominal_velocity_limit, double degraded_threshold)
: nominal_velocity_limit_(nominal_velocity_limit),
  degraded_threshold_(degraded_threshold)
{
}

SetpointCommand OffboardSupervisor::step(
  double authority_scale,
  const std::string & fault_label,
  const std::array<double, 3> & current_position,
  const std::array<double, 3> & nominal_setpoint,
  double yaw)
{
  const double authority = std::clamp(authority_scale, 0.0, 1.0);

  SetpointCommand command;
  command.yaw = yaw;
  command.velocity_limit = nominal_velocity_limit_ * authority;

  if (fault_label == "suspected_attack" || authority <= 0.15) {
    mode_ = SupervisorMode::DEGRADED_HOLD;
    command.position = current_position;
    command.velocity_limit = 0.0;
    command.hold = true;
    return command;
  }

  if (authority < degraded_threshold_ || fault_label == "degraded") {
    mode_ = SupervisorMode::RECOVERING;
    command.position = current_position;
    for (std::size_t i = 0; i < command.position.size(); ++i) {
      command.position[i] += authority * (nominal_setpoint[i] - current_position[i]);
    }
    return command;
  }

  mode_ = SupervisorMode::NOMINAL;
  command.position = nominal_setpoint;
  command.velocity_limit = nominal_velocity_limit_;
  return command;
}

const char * to_string(SupervisorMode mode)
{
  switch (mode) {
    case SupervisorMode::INIT:
      return "init";
    case SupervisorMode::ARM_AND_OFFBOARD:
      return "arm_and_offboard";
    case SupervisorMode::NOMINAL:
      return "nominal";
    case SupervisorMode::DEGRADED_HOLD:
      return "degraded_hold";
    case SupervisorMode::RECOVERING:
      return "recovering";
  }
  return "unknown";
}

}  // namespace twinguard::offboard
