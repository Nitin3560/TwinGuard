#pragma once

#include <array>
#include <string>

namespace twinguard::offboard
{

enum class SupervisorMode { INIT, ARM_AND_OFFBOARD, NOMINAL, LIMITED_OPERATION, DEGRADED_HOLD };

enum class OperationContext { NORMAL, STEADY_STATE_DEGRADED, RECOVERING_FROM_HOLD };

struct SetpointCommand
{
  std::array<double, 3> position{0.0, 0.0, 0.0};
  double yaw{0.0};
  double velocity_limit{0.0};
  bool hold{false};
};

struct SupervisorThresholds
{
  double nominal_exit_threshold{0.75};
  double nominal_enter_threshold{0.85};
  double hold_threshold{0.25};
  double hold_exit_threshold{0.45};
  double transition_dwell_s{1.0};
};

struct CircleMissionParams
{
  double center_x{0.0};
  double center_y{0.0};
  double altitude_m{2.0};
  double radius_m{3.0};
  double period_s{18.0};
  std::string mode{"circle"};
};

class OffboardSupervisor
{
public:
  OffboardSupervisor(
    double nominal_velocity_limit = 3.0,
    double degraded_threshold = 0.5,
    SupervisorThresholds thresholds = {});

  SetpointCommand step(
    double authority_scale,
    const std::string & fault_label,
    const std::array<double, 3> & current_position,
    const std::array<double, 3> & nominal_setpoint,
    double yaw = 0.0);

  SetpointCommand step(
    double target_authority,
    double applied_authority,
    const std::string & fault_label,
    const std::array<double, 3> & current_position,
    const std::array<double, 3> & nominal_setpoint,
    double yaw,
    double dt_s);

  SupervisorMode mode() const { return mode_; }
  OperationContext operation_context() const { return operation_context_; }
  double commanded_authority() const { return commanded_authority_; }

private:
  void update_state(double target_authority, const std::string & fault_label, double dt_s);
  void enter_hold(const std::array<double, 3> & current_position);
  static bool is_hard_fault(const std::string & fault_label);

  double nominal_velocity_limit_;
  double degraded_threshold_;
  SupervisorThresholds thresholds_;
  SupervisorMode mode_{SupervisorMode::INIT};
  OperationContext operation_context_{OperationContext::NORMAL};
  std::array<double, 3> hold_position_{0.0, 0.0, 0.0};
  double low_authority_time_s_{0.0};
  double hold_exit_time_s_{0.0};
  double nominal_reentry_time_s_{0.0};
  double commanded_authority_{1.0};
};

const char * to_string(SupervisorMode mode);
const char * to_string(OperationContext context);

std::array<double, 3> circle_mission_setpoint(
  const CircleMissionParams & params,
  double elapsed_s,
  double authority_scale);

double circle_mission_yaw(
  const CircleMissionParams & params,
  double elapsed_s,
  double authority_scale);

}  // namespace twinguard::offboard
