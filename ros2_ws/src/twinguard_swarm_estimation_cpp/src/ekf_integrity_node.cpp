#include "twinguard_swarm_estimation_cpp/ekf_estimator.hpp"
#include "twinguard_swarm_estimation_cpp/estimation_authority_model.hpp"
#include "twinguard_swarm_integrity_cpp/authority_aggregator.hpp"
#include "twinguard_swarm_integrity_cpp/hard_safety_monitor.hpp"
#include "twinguard_swarm_integrity_cpp/trust_scorer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace twinguard::estimation
{

class EkfIntegrityNode : public rclcpp::Node
{
public:
  EkfIntegrityNode()
  : Node("ekf_integrity_node")
  {
    drone_id_ = declare_parameter<int>("drone_id", 0);
    stale_timeout_ms_ = declare_parameter<int>("stale_timeout_ms", 500);
    process_noise_std_ = declare_parameter<double>("process_noise_std", 0.5);
    px4_position_noise_std_ = declare_parameter<double>("px4_position_noise_std", 0.25);
    base_vo_noise_std_ = declare_parameter<double>("base_vo_noise_std", 0.5);
    min_authority_ = declare_parameter<double>("min_authority", 0.15);
    communication_factor_ = declare_parameter<double>("communication_factor", 1.0);
    battery_factor_ = declare_parameter<double>("battery_factor", 1.0);
    proximity_factor_ = declare_parameter<double>("proximity_factor", 1.0);
    EstimationAuthorityConfig authority_config;
    authority_config.max_position_sigma_m =
      declare_parameter<double>("authority_max_position_sigma_m", 1.0);
    authority_config.nis_limit = declare_parameter<double>("authority_nis_limit", 10.0);
    authority_config.stale_timeout_ms = static_cast<double>(stale_timeout_ms_);
    required_sensor_mask_ = static_cast<std::uint32_t>(
      declare_parameter<int>("required_sensor_mask", SENSOR_PX4_ODOMETRY));
    twinguard::integrity::AuthorityDynamicsConfig dynamics_config;
    dynamics_config.fast_fall_rate_per_s =
      declare_parameter<double>("authority_fast_fall_rate_per_s", 4.0);
    dynamics_config.slow_rise_rate_per_s =
      declare_parameter<double>("authority_slow_rise_rate_per_s", 0.5);
    ekf_ = EkfEstimator(process_noise_std_);
    authority_model_ = EstimationAuthorityModel(authority_config);
    authority_aggregator_ = twinguard::integrity::AuthorityAggregator(dynamics_config);
    scorer_ = twinguard::integrity::TrustScorer(1.2, 0.90, min_authority_);

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "integrity_diagnostics", 10);
    trust_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("trust_state", 10);

    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        handle_odometry(*msg);
      });
    vo_diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "visual_odometry_diagnostics",
      10,
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        handle_visual_odometry_diagnostics(*msg);
      });

    timer_ = create_wall_timer(100ms, [this]() { publish_score(); });
  }

private:
  static diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
  }

  void handle_odometry(const px4_msgs::msg::VehicleOdometry & msg)
  {
    measured_position_ = {
      static_cast<double>(msg.position[0]),
      static_cast<double>(msg.position[1]),
      static_cast<double>(msg.position[2]),
    };
    measured_velocity_ = {
      static_cast<double>(msg.velocity[0]),
      static_cast<double>(msg.velocity[1]),
      static_cast<double>(msg.velocity[2]),
    };

    const auto now = get_clock()->now();
    if (!has_odometry_) {
      ekf_.reset(measured_position_, measured_velocity_);
      has_odometry_ = true;
      last_prediction_time_ = now;
    } else {
      const double dt_s = std::max((now - last_prediction_time_).seconds(), 1e-3);
      ekf_.predict(dt_s);
      last_prediction_time_ = now;
    }

    latest_position_nis_ = ekf_.update_position(measured_position_, px4_position_noise_std_);
    last_odometry_time_ = now;
  }

  void handle_visual_odometry_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    if (msg.status.empty()) {
      return;
    }

    double quality = latest_vo_quality_;
    std::array<double, 3> velocity{0.0, 0.0, 0.0};
    bool has_quality = false;
    bool has_velocity_x = false;
    bool has_velocity_y = false;
    bool has_velocity_z = false;

    for (const auto & value : msg.status.front().values) {
      try {
        if (value.key == "quality") {
          latest_vo_quality_ = std::clamp(std::stod(value.value), 0.0, 1.0);
          quality = latest_vo_quality_;
          has_quality = true;
        } else if (value.key == "velocity_x") {
          velocity[0] = std::stod(value.value);
          has_velocity_x = true;
        } else if (value.key == "velocity_y") {
          velocity[1] = std::stod(value.value);
          has_velocity_y = true;
        } else if (value.key == "velocity_z") {
          velocity[2] = std::stod(value.value);
          has_velocity_z = true;
        }
      } catch (const std::exception &) {
        latest_vo_quality_ = 0.0;
        return;
      }
    }

    const bool has_velocity = has_velocity_x && has_velocity_y && has_velocity_z;
    if (!has_odometry_ || msg.status.front().message != "visual_odometry_active" ||
      !has_quality || !has_velocity)
    {
      return;
    }

    const double vo_noise_std = base_vo_noise_std_ / std::max(quality, 0.05);
    latest_vo_nis_ = ekf_.update_velocity(velocity, vo_noise_std);
    last_vo_time_ = get_clock()->now();
    has_visual_odometry_ = true;
  }

  void publish_score()
  {
    const auto now = get_clock()->now();
    const int64_t age_ms = has_odometry_ ?
      (now - last_odometry_time_).nanoseconds() / 1000000 : 0;

    const auto & state = ekf_.state();
    twinguard::integrity::HardSafetyInput hard_input;
    hard_input.authority_floor = min_authority_;
    hard_input.has_valid_localization_source = has_odometry_;
    hard_input.estimator_stale = has_odometry_ && age_ms > stale_timeout_ms_;
    hard_input.covariance_required = true;
    hard_input.state_values.reserve(state.x.size());
    for (int i = 0; i < state.x.size(); ++i) {
      hard_input.state_values.push_back(state.x(i));
    }
    hard_input.covariance_values.reserve(state.P.size());
    for (int row = 0; row < state.P.rows(); ++row) {
      for (int col = 0; col < state.P.cols(); ++col) {
        hard_input.covariance_values.push_back(state.P(row, col));
      }
    }

    const auto hard_result = hard_safety_monitor_.evaluate(hard_input);
    if (hard_result.hard_override_active) {
      publish_hard_override(now, age_ms, hard_result);
      return;
    }

    const std::array<double, 3> fused_position{
      state.x(0),
      state.x(1),
      state.x(2),
    };
    const auto trust = scorer_.update(measured_position_, fused_position);
    const auto authority = authority_model_.evaluate(make_authority_input(state, age_ms, now));
    auto factors = make_system_factors(authority.estimation_factor);
    const auto system_authority = authority_aggregator_.update(
      factors,
      applied_authority_,
      authority_dt(now));
    applied_authority_ = system_authority.applied_authority;
    last_authority_time_ = now;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "twinguard_ekf_integrity_drone_" + std::to_string(drone_id_);
    status.hardware_id = "uav_" + std::to_string(drone_id_);
    status.level = trust.fault_label == "nominal" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = trust.fault_label;
    status.values = {
      kv("trust", std::to_string(trust.trust)),
      kv("residual", std::to_string(trust.residual)),
      kv("authority_scale", std::to_string(system_authority.applied_authority)),
      kv("target_authority", std::to_string(system_authority.target_authority)),
      kv("applied_authority", std::to_string(system_authority.applied_authority)),
      kv("active_limiting_factor", system_authority.active_limiting_factor),
      kv("residual_authority_scale", std::to_string(trust.authority_scale)),
      kv("estimation_factor", std::to_string(authority.estimation_factor)),
      kv("communication_factor", std::to_string(factors.communication_factor)),
      kv("battery_factor", std::to_string(factors.battery_factor)),
      kv("proximity_factor", std::to_string(factors.proximity_factor)),
      kv("covariance_factor", std::to_string(authority.covariance_factor)),
      kv("nis_factor", std::to_string(authority.nis_factor)),
      kv("freshness_factor", std::to_string(authority.freshness_factor)),
      kv("availability_factor", std::to_string(authority.availability_factor)),
      kv("estimation_limiting_reason", authority.estimation_limiting_reason),
      kv("position_sigma_m", std::to_string(authority.position_sigma_m)),
      kv("odometry_age_ms", std::to_string(age_ms)),
      kv("position_nis", std::to_string(latest_position_nis_)),
      kv("velocity_nis", std::to_string(latest_vo_nis_)),
      kv("visual_odometry_nis", std::to_string(latest_vo_nis_)),
      kv("visual_odometry_quality", std::to_string(latest_vo_quality_)),
      kv("visual_odometry_active", has_visual_odometry_ ? "true" : "false"),
      kv("measurement_age_ms", std::to_string(authority.measurement_age_ms)),
      kv("active_sensor_mask", std::to_string(authority.active_sensor_mask)),
    };

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now;
    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);

    geometry_msgs::msg::PointStamped trust_state;
    trust_state.header.stamp = diagnostics.header.stamp;
    trust_state.header.frame_id = "map";
    trust_state.point.x = trust.trust;
    trust_state.point.y = trust.residual;
    trust_state.point.z = system_authority.applied_authority;
    trust_pub_->publish(trust_state);
  }

  void publish_waiting_diagnostic(const std::string & message)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "twinguard_ekf_integrity_drone_" + std::to_string(drone_id_);
    status.hardware_id = "uav_" + std::to_string(drone_id_);
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = message;

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now();
    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);
  }

  void publish_hard_override(
    const rclcpp::Time & stamp,
    int64_t age_ms,
    const twinguard::integrity::HardSafetyResult & hard_result)
  {
    const auto authority = authority_model_.evaluate(make_authority_input(ekf_.state(), age_ms, stamp));
    applied_authority_ = hard_result.target_authority;
    last_authority_time_ = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "twinguard_ekf_integrity_drone_" + std::to_string(drone_id_);
    status.hardware_id = "uav_" + std::to_string(drone_id_);
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "hard_safety_override";
    status.values = {
      kv("hard_override_active", "true"),
      kv("hard_override_reason", hard_result.diagnostic_reason),
      kv("integrity_state", twinguard::integrity::to_string(hard_result.integrity_state)),
      kv("operation_context", twinguard::integrity::to_string(hard_result.operation_context)),
      kv("transition_reason", twinguard::integrity::to_string(hard_result.transition_reason)),
      kv("target_authority", std::to_string(hard_result.target_authority)),
      kv("authority_scale", std::to_string(hard_result.target_authority)),
      kv("applied_authority", std::to_string(hard_result.target_authority)),
      kv("active_limiting_factor", "hard_safety"),
      kv("estimation_factor", std::to_string(authority.estimation_factor)),
      kv("communication_factor", std::to_string(communication_factor_)),
      kv("battery_factor", std::to_string(battery_factor_)),
      kv("proximity_factor", std::to_string(proximity_factor_)),
      kv("covariance_factor", std::to_string(authority.covariance_factor)),
      kv("nis_factor", std::to_string(authority.nis_factor)),
      kv("freshness_factor", std::to_string(authority.freshness_factor)),
      kv("availability_factor", std::to_string(authority.availability_factor)),
      kv("estimation_limiting_reason", authority.estimation_limiting_reason),
      kv("position_sigma_m", std::to_string(authority.position_sigma_m)),
      kv("odometry_age_ms", std::to_string(age_ms)),
      kv("position_nis", std::to_string(latest_position_nis_)),
      kv("velocity_nis", std::to_string(latest_vo_nis_)),
      kv("visual_odometry_nis", std::to_string(latest_vo_nis_)),
      kv("visual_odometry_quality", std::to_string(latest_vo_quality_)),
      kv("visual_odometry_active", has_visual_odometry_ ? "true" : "false"),
      kv("measurement_age_ms", std::to_string(authority.measurement_age_ms)),
      kv("active_sensor_mask", std::to_string(authority.active_sensor_mask)),
    };

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = stamp;
    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);

    geometry_msgs::msg::PointStamped trust_state;
    trust_state.header.stamp = diagnostics.header.stamp;
    trust_state.header.frame_id = "map";
    trust_state.point.x = 0.0;
    trust_state.point.y = 0.0;
    trust_state.point.z = hard_result.target_authority;
    trust_pub_->publish(trust_state);
  }

  EstimationAuthorityInput make_authority_input(
    const EkfState & state,
    int64_t age_ms,
    const rclcpp::Time & now) const
  {
    EstimationAuthorityInput input;
    input.position_variance = {
      state.P(0, 0),
      state.P(1, 1),
      state.P(2, 2),
    };
    input.position_nis = latest_position_nis_;
    input.velocity_nis = latest_vo_nis_;
    input.measurement_age_ms = static_cast<double>(age_ms);
    input.required_sensor_mask = required_sensor_mask_;
    input.active_sensor_mask = active_sensor_mask(now);
    return input;
  }

  twinguard::integrity::AuthorityFactors make_system_factors(double estimation_factor) const
  {
    twinguard::integrity::AuthorityFactors factors;
    factors.estimation_factor = estimation_factor;
    factors.communication_factor = communication_factor_;
    factors.battery_factor = battery_factor_;
    factors.proximity_factor = proximity_factor_;
    return factors;
  }

  double authority_dt(const rclcpp::Time & now) const
  {
    if (last_authority_time_.nanoseconds() == 0) {
      return 0.0;
    }
    return std::max((now - last_authority_time_).seconds(), 0.0);
  }

  std::uint32_t active_sensor_mask(const rclcpp::Time & now) const
  {
    std::uint32_t mask = 0u;
    if (has_odometry_) {
      mask |= SENSOR_PX4_ODOMETRY;
    }
    if (has_visual_odometry_ &&
      (now - last_vo_time_).nanoseconds() / 1000000 <= stale_timeout_ms_)
    {
      mask |= SENSOR_VISUAL_ODOMETRY;
    }
    return mask;
  }

  int drone_id_{0};
  int stale_timeout_ms_{500};
  double process_noise_std_{0.5};
  double px4_position_noise_std_{0.25};
  double base_vo_noise_std_{0.5};
  double min_authority_{0.15};
  double communication_factor_{1.0};
  double battery_factor_{1.0};
  double proximity_factor_{1.0};
  std::uint32_t required_sensor_mask_{SENSOR_PX4_ODOMETRY};
  double applied_authority_{1.0};
  double latest_position_nis_{0.0};
  double latest_vo_nis_{0.0};
  double latest_vo_quality_{0.0};
  bool has_odometry_{false};
  bool has_visual_odometry_{false};
  std::array<double, 3> measured_position_{0.0, 0.0, 0.0};
  std::array<double, 3> measured_velocity_{0.0, 0.0, 0.0};
  rclcpp::Time last_prediction_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_vo_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_authority_time_{0, 0, RCL_ROS_TIME};
  EkfEstimator ekf_{0.5};
  EstimationAuthorityModel authority_model_;
  twinguard::integrity::AuthorityAggregator authority_aggregator_;
  twinguard::integrity::HardSafetyMonitor hard_safety_monitor_;
  twinguard::integrity::TrustScorer scorer_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr vo_diag_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr trust_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace twinguard::estimation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<twinguard::estimation::EkfIntegrityNode>());
  rclcpp::shutdown();
  return 0;
}
