#include "twinguard_swarm_integrity_cpp/offboard_supervisor.hpp"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace twinguard::offboard
{

class FormationSupervisorNode : public rclcpp::Node
{
public:
  FormationSupervisorNode()
  : Node("formation_supervisor_node")
  {
    drone_id_ = declare_parameter<int>("drone_id", 0);
    target_system_ = declare_parameter<int>("target_system", drone_id_ + 1);
    nominal_setpoint_ = {
      declare_parameter<double>("nominal_x_m", 0.0),
      declare_parameter<double>("nominal_y_m", 0.0),
      declare_parameter<double>("nominal_z_m", -2.0),
    };
    nominal_yaw_ = declare_parameter<double>("nominal_yaw_rad", 0.0);
    nominal_velocity_limit_ = declare_parameter<double>("nominal_velocity_limit_mps", 3.0);
    degraded_threshold_ = declare_parameter<double>("degraded_threshold", 0.5);
    stale_timeout_ms_ = declare_parameter<int>("stale_timeout_ms", 500);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    force_arm_ = declare_parameter<bool>("force_arm", false);

    supervisor_ = OffboardSupervisor(nominal_velocity_limit_, degraded_threshold_);

    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "fmu/in/vehicle_command", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "supervisor_diagnostics", 10);

    trust_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "trust_state",
      10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        handle_trust_state(*msg);
      });

    diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "integrity_diagnostics",
      10,
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        handle_integrity_diagnostics(*msg);
      });

    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        handle_odometry(*msg);
      });

    const double setpoint_rate_hz = declare_parameter<double>("setpoint_rate_hz", 10.0);
    const auto period = std::chrono::duration<double>(1.0 / std::max(setpoint_rate_hz, 2.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });

    RCLCPP_INFO(
      get_logger(),
      "TwinGuard C++ formation supervisor ready for drone_%d, target_system=%d",
      drone_id_,
      target_system_);
  }

private:
  static diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
  }

  int64_t timestamp_us() const
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  void handle_trust_state(const geometry_msgs::msg::PointStamped & msg)
  {
    trust_ = std::clamp(msg.point.x, 0.0, 1.0);
    residual_ = std::max(0.0, msg.point.y);
    authority_scale_ = std::clamp(msg.point.z, 0.0, 1.0);

    if (trust_ < 0.35 && residual_ > 1.0) {
      fault_label_ = "suspected_attack";
    } else if (trust_ < 0.65) {
      fault_label_ = "degraded";
    } else {
      fault_label_ = "nominal";
    }
  }

  void handle_integrity_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    if (!msg.status.empty() && !msg.status.front().message.empty()) {
      fault_label_ = msg.status.front().message;
    }
  }

  void handle_odometry(const px4_msgs::msg::VehicleOdometry & msg)
  {
    current_position_ = {
      static_cast<double>(msg.position[0]),
      static_cast<double>(msg.position[1]),
      static_cast<double>(msg.position[2]),
    };
    has_odometry_ = true;
    last_odometry_time_ = get_clock()->now();
  }

  void tick()
  {
    ++tick_count_;
    const int64_t timestamp = timestamp_us();
    const auto now = get_clock()->now();
    const bool odometry_stale = !has_odometry_ ||
      ((now - last_odometry_time_).nanoseconds() / 1000000 > stale_timeout_ms_);

    const std::array<double, 3> current = odometry_stale ? nominal_setpoint_ : current_position_;
    const std::string fault = odometry_stale ? "suspected_attack" : fault_label_;
    const double authority = odometry_stale ? 0.15 : authority_scale_;
    const SetpointCommand command = supervisor_.step(
      authority,
      fault,
      current,
      nominal_setpoint_,
      nominal_yaw_);

    publish_offboard_mode(timestamp);
    publish_setpoint(timestamp, command);

    if (auto_arm_ && (tick_count_ == 20 || tick_count_ == 60)) {
      request_offboard_mode(timestamp);
      arm(timestamp);
    }

    publish_diagnostics(timestamp, command, odometry_stale);
  }

  void publish_offboard_mode(int64_t timestamp)
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = timestamp;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    offboard_mode_pub_->publish(msg);
  }

  void publish_setpoint(int64_t timestamp, const SetpointCommand & command)
  {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = timestamp;
    msg.position = {
      static_cast<float>(command.position[0]),
      static_cast<float>(command.position[1]),
      static_cast<float>(command.position[2]),
    };
    msg.velocity = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
    };
    msg.acceleration = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
    };
    msg.jerk = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
    };
    msg.yaw = static_cast<float>(command.yaw);
    msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
    setpoint_pub_->publish(msg);
  }

  void request_offboard_mode(int64_t timestamp)
  {
    publish_vehicle_command(
      timestamp,
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0,
      6.0);
    offboard_requested_ = true;
  }

  void arm(int64_t timestamp)
  {
    publish_vehicle_command(
      timestamp,
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0,
      force_arm_ ? 21196.0 : 0.0);
    arm_requested_ = true;
  }

  void publish_vehicle_command(
    int64_t timestamp,
    uint32_t command,
    double param1 = 0.0,
    double param2 = 0.0)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = timestamp;
    msg.param1 = static_cast<float>(param1);
    msg.param2 = static_cast<float>(param2);
    msg.command = command;
    msg.target_system = static_cast<uint8_t>(target_system_);
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void publish_diagnostics(
    int64_t timestamp,
    const SetpointCommand & command,
    bool odometry_stale)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "twinguard/formation_supervisor_drone_" + std::to_string(drone_id_);
    status.hardware_id = "uav_" + std::to_string(drone_id_);
    status.level = odometry_stale || command.hold ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = to_string(supervisor_.mode());
    status.values = {
      kv("timestamp_us", std::to_string(timestamp)),
      kv("trust", std::to_string(trust_)),
      kv("residual", std::to_string(residual_)),
      kv("authority_scale", std::to_string(authority_scale_)),
      kv("fault_label", fault_label_),
      kv("hold", command.hold ? "true" : "false"),
      kv("velocity_limit_mps", std::to_string(command.velocity_limit)),
      kv("setpoint_x", std::to_string(command.position[0])),
      kv("setpoint_y", std::to_string(command.position[1])),
      kv("setpoint_z", std::to_string(command.position[2])),
      kv("odometry_stale", odometry_stale ? "true" : "false"),
      kv("offboard_requested", offboard_requested_ ? "true" : "false"),
      kv("arm_requested", arm_requested_ ? "true" : "false"),
    };

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now();
    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);
  }

  int drone_id_{0};
  int target_system_{1};
  int stale_timeout_ms_{500};
  int tick_count_{0};
  double nominal_yaw_{0.0};
  double nominal_velocity_limit_{3.0};
  double degraded_threshold_{0.5};
  double trust_{1.0};
  double residual_{0.0};
  double authority_scale_{1.0};
  bool has_odometry_{false};
  bool auto_arm_{false};
  bool force_arm_{false};
  bool offboard_requested_{false};
  bool arm_requested_{false};
  std::string fault_label_{"nominal"};
  std::array<double, 3> nominal_setpoint_{0.0, 0.0, -2.0};
  std::array<double, 3> current_position_{0.0, 0.0, 0.0};
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  OffboardSupervisor supervisor_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr trust_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace twinguard::offboard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<twinguard::offboard::FormationSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
