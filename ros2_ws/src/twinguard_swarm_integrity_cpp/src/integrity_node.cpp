#include "twinguard_swarm_integrity_cpp/trust_scorer.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace twinguard::integrity
{

class IntegrityNode : public rclcpp::Node
{
public:
  IntegrityNode()
  : Node("integrity_node_cpp")
  {
    drone_id_ = declare_parameter<int>("drone_id", 0);
    stale_timeout_ms_ = declare_parameter<int>("stale_timeout_ms", 500);
    prediction_dt_ = declare_parameter<double>("prediction_dt", 0.1);
    twin_ = DigitalTwinPredictor(prediction_dt_);
    const double alpha = declare_parameter<double>("alpha", 1.2);
    const double beta = declare_parameter<double>("beta", 0.90);
    const double min_authority = declare_parameter<double>("min_authority", 0.15);
    scorer_ = TrustScorer(alpha, beta, min_authority);

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "integrity_diagnostics", 10);
    trust_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "trust_state", 10);

    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        handle_odometry(*msg);
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

    if (!has_odometry_) {
      twin_.reset(measured_position_, measured_velocity_);
      has_odometry_ = true;
    } else {
      twin_.correct_velocity(measured_velocity_);
    }

    last_odometry_time_ = get_clock()->now();
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

    const auto predicted = twin_.predict();
    const TrustState state = scorer_.update(measured_position_, predicted);
    constexpr double kBaseCorrectionGain = 0.15;
    twin_.correct_position(measured_position_, kBaseCorrectionGain * state.trust);

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "twinguard_integrity_drone_" + std::to_string(drone_id_);
    status.hardware_id = "uav_" + std::to_string(drone_id_);
    status.level = state.fault_label == "nominal" ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = state.fault_label;
    status.values.push_back(kv("trust", std::to_string(state.trust)));
    status.values.push_back(kv("residual", std::to_string(state.residual)));
    status.values.push_back(kv("authority_scale", std::to_string(state.authority_scale)));
    status.values.push_back(kv("odometry_age_ms", std::to_string(age_ms)));

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now;
    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);

    geometry_msgs::msg::PointStamped trust_state;
    trust_state.header.stamp = diagnostics.header.stamp;
    trust_state.header.frame_id = "map";
    trust_state.point.x = state.trust;
    trust_state.point.y = state.residual;
    trust_state.point.z = state.authority_scale;
    trust_pub_->publish(trust_state);
  }

  void publish_waiting_diagnostic(const std::string & message)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "twinguard_integrity_drone_" + std::to_string(drone_id_);
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
  double prediction_dt_{0.1};
  bool has_odometry_{false};
  std::array<double, 3> measured_position_{0.0, 0.0, 0.0};
  std::array<double, 3> measured_velocity_{0.0, 0.0, 0.0};
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  DigitalTwinPredictor twin_{0.1};
  TrustScorer scorer_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr trust_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace twinguard::integrity

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<twinguard::integrity::IntegrityNode>());
  rclcpp::shutdown();
  return 0;
}
