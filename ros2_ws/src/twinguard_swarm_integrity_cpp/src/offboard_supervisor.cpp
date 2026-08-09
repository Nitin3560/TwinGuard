#include "twinguard_swarm_integrity_cpp/offboard_supervisor.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace twinguard::offboard
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

OffboardSupervisor::OffboardSupervisor(
  double nominal_velocity_limit,
  double degraded_threshold,
  SupervisorThresholds thresholds)
: nominal_velocity_limit_(nominal_velocity_limit),
  degraded_threshold_(degraded_threshold),
  thresholds_(thresholds)
{
  thresholds_.nominal_exit_threshold = std::clamp(
    thresholds_.nominal_exit_threshold, degraded_threshold_, 1.0);
  thresholds_.nominal_enter_threshold = std::clamp(
    thresholds_.nominal_enter_threshold, thresholds_.nominal_exit_threshold, 1.0);
  thresholds_.hold_threshold = std::clamp(thresholds_.hold_threshold, 0.0, degraded_threshold_);
  thresholds_.hold_exit_threshold = std::clamp(
    thresholds_.hold_exit_threshold, thresholds_.hold_threshold, thresholds_.nominal_enter_threshold);
  thresholds_.transition_dwell_s = std::max(thresholds_.transition_dwell_s, 0.0);
}

SetpointCommand OffboardSupervisor::step(
  double authority_scale,
  const std::string & fault_label,
  const std::array<double, 3> & current_position,
  const std::array<double, 3> & nominal_setpoint,
  double yaw)
{
  return step(
    authority_scale,
    authority_scale,
    fault_label,
    current_position,
    nominal_setpoint,
    yaw,
    0.1);
}

SetpointCommand OffboardSupervisor::step(
  double target_authority,
  double applied_authority,
  const std::string & fault_label,
  const std::array<double, 3> & current_position,
  const std::array<double, 3> & nominal_setpoint,
  double yaw,
  double dt_s)
{
  const double target = std::clamp(target_authority, 0.0, 1.0);
  const double applied = std::clamp(applied_authority, 0.0, 1.0);
  const SupervisorMode previous_mode = mode_;
  update_state(target, fault_label, std::max(dt_s, 0.0));
  if (mode_ == SupervisorMode::DEGRADED_HOLD && previous_mode != SupervisorMode::DEGRADED_HOLD) {
    enter_hold(current_position);
  }

  SetpointCommand command;
  command.yaw = yaw;
  command.velocity_limit = nominal_velocity_limit_ * applied;

  if (mode_ == SupervisorMode::DEGRADED_HOLD) {
    commanded_authority_ = 0.0;
    command.position = hold_position_;
    command.velocity_limit = 0.0;
    command.hold = true;
    return command;
  }

  if (mode_ == SupervisorMode::LIMITED_OPERATION) {
    commanded_authority_ = applied;
    command.position = current_position;
    for (std::size_t i = 0; i < command.position.size(); ++i) {
      command.position[i] += applied * (nominal_setpoint[i] - current_position[i]);
    }
    return command;
  }

  mode_ = SupervisorMode::NOMINAL;
  operation_context_ = OperationContext::NORMAL;
  commanded_authority_ = 1.0;
  command.position = nominal_setpoint;
  command.velocity_limit = nominal_velocity_limit_;
  return command;
}

void OffboardSupervisor::update_state(
  double target_authority,
  const std::string & fault_label,
  double dt_s)
{
  if (mode_ == SupervisorMode::INIT || mode_ == SupervisorMode::ARM_AND_OFFBOARD) {
    mode_ = SupervisorMode::NOMINAL;
  }

  if (is_hard_fault(fault_label)) {
    mode_ = SupervisorMode::DEGRADED_HOLD;
    operation_context_ = OperationContext::STEADY_STATE_DEGRADED;
    low_authority_time_s_ = 0.0;
    hold_exit_time_s_ = 0.0;
    nominal_reentry_time_s_ = 0.0;
    return;
  }

  if (mode_ == SupervisorMode::NOMINAL) {
    if (target_authority < thresholds_.nominal_exit_threshold || fault_label == "degraded") {
      low_authority_time_s_ += dt_s;
      if (low_authority_time_s_ >= thresholds_.transition_dwell_s) {
        mode_ = SupervisorMode::LIMITED_OPERATION;
        operation_context_ = OperationContext::STEADY_STATE_DEGRADED;
        nominal_reentry_time_s_ = 0.0;
      }
    } else {
      low_authority_time_s_ = 0.0;
    }
    return;
  }

  if (mode_ == SupervisorMode::LIMITED_OPERATION) {
    low_authority_time_s_ = 0.0;
    hold_exit_time_s_ = 0.0;
    if (target_authority < thresholds_.hold_threshold) {
      mode_ = SupervisorMode::DEGRADED_HOLD;
      operation_context_ = OperationContext::STEADY_STATE_DEGRADED;
      nominal_reentry_time_s_ = 0.0;
      return;
    }

    if (target_authority > thresholds_.nominal_enter_threshold && fault_label == "nominal") {
      nominal_reentry_time_s_ += dt_s;
      if (nominal_reentry_time_s_ >= thresholds_.transition_dwell_s) {
        mode_ = SupervisorMode::NOMINAL;
        operation_context_ = OperationContext::NORMAL;
        nominal_reentry_time_s_ = 0.0;
      }
    } else {
      nominal_reentry_time_s_ = 0.0;
    }
    return;
  }

  if (mode_ == SupervisorMode::DEGRADED_HOLD) {
    low_authority_time_s_ = 0.0;
    nominal_reentry_time_s_ = 0.0;
    if (target_authority > thresholds_.hold_exit_threshold && fault_label != "suspected_attack") {
      hold_exit_time_s_ += dt_s;
      if (hold_exit_time_s_ >= thresholds_.transition_dwell_s) {
        mode_ = SupervisorMode::LIMITED_OPERATION;
        operation_context_ = OperationContext::RECOVERING_FROM_HOLD;
        hold_exit_time_s_ = 0.0;
      }
    } else {
      hold_exit_time_s_ = 0.0;
    }
  }
}

void OffboardSupervisor::enter_hold(const std::array<double, 3> & current_position)
{
  hold_position_ = current_position;
}

bool OffboardSupervisor::is_hard_fault(const std::string & fault_label)
{
  return fault_label == "hard_safety_override";
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
    case SupervisorMode::LIMITED_OPERATION:
      return "limited_operation";
    case SupervisorMode::DEGRADED_HOLD:
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

std::array<double, 3> circle_mission_setpoint(
  const CircleMissionParams & params,
  double elapsed_s,
  double authority_scale)
{
  if (params.mode == "hold") {
    return {params.center_x, params.center_y, -params.altitude_m};
  }

  const double amplitude = params.radius_m * std::clamp(authority_scale, 0.0, 1.0);
  const double phase = 2.0 * kPi * elapsed_s / std::max(params.period_s, 1.0);

  if (params.mode == "figure8") {
    const double x = params.center_x + amplitude * std::sin(phase);
    const double y = params.center_y + amplitude * std::sin(phase) * std::cos(phase);
    return {x, y, -params.altitude_m};
  }

  const double x = params.center_x + amplitude * std::cos(phase);
  const double y = params.center_y + amplitude * std::sin(phase);
  return {x, y, -params.altitude_m};
}

double circle_mission_yaw(
  const CircleMissionParams & params,
  double elapsed_s,
  double authority_scale)
{
  if (params.mode == "hold") {
    return 0.0;
  }

  const auto position = circle_mission_setpoint(params, elapsed_s, authority_scale);
  return std::atan2(position[1] - params.center_y, position[0] - params.center_x) + kPi / 2.0;
}

}  // namespace twinguard::offboard
