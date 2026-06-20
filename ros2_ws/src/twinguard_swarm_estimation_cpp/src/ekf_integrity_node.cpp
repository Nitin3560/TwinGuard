#include "twinguard_swarm_estimation_cpp/ekf_estimator.hpp"
#include "twinguard_swarm_integrity_cpp/trust_scorer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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
    ekf_ = EkfEstimator(process_noise_std_);

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "integrity_diagnostics", 10);
    trust_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("trust_state", 10);

    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        handle_odometry(*msg);
      });
    vo_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "visual_odometry",
      10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        handle_visual_odometry(*msg);
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

  void handle_visual_odometry(const geometry_msgs::msg::TwistStamped & msg)
  {
    if (!has_odometry_) {
      return;
    }

    const std::array<double, 3> velocity{
      msg.twist.linear.x,
      msg.twist.linear.y,
      msg.twist.linear.z,
    };
    const double vo_noise_std = base_vo_noise_std_ / std::max(latest_vo_quality_, 0.05);
    latest_vo_nis_ = ekf_.update_velocity(velocity, vo_noise_std);
    last_vo_time_ = get_clock()->now();
    has_visual_odometry_ = true;
  }

  void handle_visual_odometry_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    if (msg.status.empty()) {
      return;
    }
    for (const auto & value : msg.status.front().values) {
      if (value.key == "quality") {
        try {
          latest_vo_quality_ = std::clamp(std::stod(value.value), 0.0, 1.0);
        } catch (const std::exception &) {
          latest_vo_quality_ = 0.0;
        }
      }
    }
  }

  void publish_score()
  {
    if (!has_odometry_) {
      publish_waiting_diagnostic("waiting_for_px4_vehicle_odometry");
      return;
    }

    const auto now = get_clock()->now();
    const int64_t age_ms = (now - last_odometry_time_).nanoseconds() / 1000000;
    if (age_ms > stale_timeout_ms_) {
      publish_waiting_diagnostic("stale_px4_vehicle_odometry");
      return;
    }

    const auto & state = ekf_.state();
    const std::array<double, 3> fused_position{
      state.x(0),
      state.x(1),
      state.x(2),
    };
    const auto trust = scorer_.update(measured_position_, fused_position);

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
      kv("authority_scale", std::to_string(trust.authority_scale)),
      kv("odometry_age_ms", std::to_string(age_ms)),
      kv("position_nis", std::to_string(latest_position_nis_)),
      kv("visual_odometry_nis", std::to_string(latest_vo_nis_)),
      kv("visual_odometry_quality", std::to_string(latest_vo_quality_)),
      kv("visual_odometry_active", has_visual_odometry_ ? "true" : "false"),
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
    trust_state.point.z = trust.authority_scale;
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

  int drone_id_{0};
  int stale_timeout_ms_{500};
  double process_noise_std_{0.5};
  double px4_position_noise_std_{0.25};
  double base_vo_noise_std_{0.5};
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
  EkfEstimator ekf_{0.5};
  twinguard::integrity::TrustScorer scorer_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vo_sub_;
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
