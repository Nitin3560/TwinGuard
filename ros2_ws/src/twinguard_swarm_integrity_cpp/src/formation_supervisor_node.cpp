#include "twinguard_swarm_integrity_cpp/offboard_supervisor.hpp"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "twinguard_swarm_planning_cpp/astar_planner.hpp"
#include "twinguard_swarm_planning_cpp/bt_nodes.hpp"

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
    min_authority_scale_ = declare_parameter<double>("min_authority_scale", 0.15);
    SupervisorThresholds thresholds;
    thresholds.nominal_exit_threshold = declare_parameter<double>("nominal_exit_threshold", 0.75);
    thresholds.nominal_enter_threshold = declare_parameter<double>("nominal_enter_threshold", 0.85);
    thresholds.hold_threshold = declare_parameter<double>("hold_threshold", 0.25);
    thresholds.hold_exit_threshold = declare_parameter<double>("hold_exit_threshold", 0.45);
    thresholds.transition_dwell_s = declare_parameter<double>("transition_dwell_s", 1.0);
    stale_timeout_ms_ = declare_parameter<int>("stale_timeout_ms", 500);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    force_arm_ = declare_parameter<bool>("force_arm", false);
    mission_params_.mode = declare_parameter<std::string>("mission_mode", "hold");
    mission_params_.radius_m = declare_parameter<double>("mission_radius_m", 0.0);
    mission_params_.period_s = declare_parameter<double>("mission_period_s", 18.0);
    mission_params_.center_x = declare_parameter<double>("center_x_m", nominal_setpoint_[0]);
    mission_params_.center_y = declare_parameter<double>("center_y_m", nominal_setpoint_[1]);
    mission_params_.altitude_m = std::abs(nominal_setpoint_[2]);
    start_time_ = get_clock()->now();
    const bool obstacle_enabled = declare_parameter<bool>("static_obstacle_enabled", false);
    if (obstacle_enabled) {
      obstacles_.push_back(twinguard::planning::Obstacle{
        {
          declare_parameter<double>("static_obstacle_x_m", 0.0),
          declare_parameter<double>("static_obstacle_y_m", 0.0),
          declare_parameter<double>("static_obstacle_z_m", -2.0),
        },
        declare_parameter<double>("static_obstacle_radius_m", 1.0),
      });
    }
    const std::string default_tree_path =
      ament_index_cpp::get_package_share_directory("twinguard_swarm_planning_cpp") +
      "/trees/twinguard_mission.xml";
    const std::string tree_path = declare_parameter<std::string>(
      "behavior_tree_xml",
      default_tree_path);
    bt_blackboard_ = BT::Blackboard::create();
    BT::BehaviorTreeFactory factory;
    twinguard::planning::registerTwinGuardBtNodes(factory);
    bt_tree_ = std::make_unique<BT::Tree>(factory.createTreeFromFile(tree_path, bt_blackboard_));

    supervisor_ = OffboardSupervisor(nominal_velocity_limit_, degraded_threshold_, thresholds);

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

  int64_t timestamp_us()
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  void handle_trust_state(const geometry_msgs::msg::PointStamped & msg)
  {
    trust_ = std::clamp(msg.point.x, 0.0, 1.0);
    residual_ = std::max(0.0, msg.point.y);
    authority_scale_ = std::clamp(msg.point.z, min_authority_scale_, 1.0);
    if (!has_target_authority_) {
      target_authority_ = authority_scale_;
    }

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
      const auto & status = msg.status.front();
      fault_label_ = status.message;
      bool saw_target_authority = false;
      bool saw_hard_override = false;
      for (const auto & item : status.values) {
        try {
          if (item.key == "target_authority") {
            target_authority_ = std::clamp(std::stod(item.value), 0.0, 1.0);
            has_target_authority_ = true;
            saw_target_authority = true;
          } else if (item.key == "applied_authority") {
            authority_scale_ = std::clamp(std::stod(item.value), min_authority_scale_, 1.0);
          } else if (item.key == "hard_override_active") {
            hard_override_active_ = (item.value == "true" || item.value == "1");
            saw_hard_override = true;
          }
        } catch (const std::exception &) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring malformed integrity diagnostic %s=%s",
            item.key.c_str(), item.value.c_str());
        }
      }
      if (!saw_hard_override) {
        hard_override_active_ = status.message == "hard_safety_override";
      }
      if (!saw_target_authority && status.message != "hard_safety_override") {
        has_target_authority_ = false;
        target_authority_ = authority_scale_;
      }
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
    double dt_s = 0.1;
    if (last_tick_time_.nanoseconds() > 0) {
      dt_s = std::max(0.0, (now - last_tick_time_).seconds());
    }
    last_tick_time_ = now;
    const bool odometry_stale = !has_odometry_ ||
      ((now - last_odometry_time_).nanoseconds() / 1000000 > stale_timeout_ms_);

    const std::array<double, 3> current = odometry_stale ? nominal_setpoint_ : current_position_;
    const bool hard_override = odometry_stale || hard_override_active_;
    const std::string fault = hard_override ? "hard_safety_override" : fault_label_;
    const double target_authority = hard_override ? min_authority_scale_ : target_authority_;
    const double applied_authority = hard_override ? min_authority_scale_ : authority_scale_;
    std::array<double, 3> mission = nominal_setpoint_;
    double mission_yaw = nominal_yaw_;

    if (supervisor_.mode() != SupervisorMode::DEGRADED_HOLD) {
      mission_elapsed_s_ += dt_s;
    }

    if (mission_params_.mode == "circle" || mission_params_.mode == "figure8") {
      mission = circle_mission_setpoint(mission_params_, mission_elapsed_s_, applied_authority);
      mission_yaw = circle_mission_yaw(mission_params_, mission_elapsed_s_, applied_authority);
    }

    std::array<double, 3> nominal = mission;
    double yaw = mission_yaw;
    if (bt_tree_ && bt_blackboard_) {
      bt_blackboard_->set("current_position", current);
      bt_blackboard_->set("fault_label", fault);
      bt_blackboard_->set("authority_scale", applied_authority);
      bt_blackboard_->set("target_authority", target_authority);
      bt_blackboard_->set("operation_context", std::string{to_string(supervisor_.operation_context())});
      bt_blackboard_->set("mission_setpoint", mission);
      bt_blackboard_->set("mission_yaw", mission_yaw);
      bt_blackboard_->set("obstacles", obstacles_);
      bt_blackboard_->set("computed_setpoint", mission);
      bt_blackboard_->set("computed_yaw", mission_yaw);
      bt_blackboard_->set("planner_mode", std::string{"mission_fallback"});
      const BT::NodeStatus tree_status = bt_tree_->tickRoot();
      if (tree_status == BT::NodeStatus::SUCCESS) {
        nominal = bt_blackboard_->get<std::array<double, 3>>("computed_setpoint");
        yaw = bt_blackboard_->get<double>("computed_yaw");
        planner_mode_ = bt_blackboard_->get<std::string>("planner_mode");
      } else {
        planner_mode_ = "mission_fallback";
      }
    }

    const SetpointCommand command = supervisor_.step(
      target_authority,
      applied_authority,
      fault,
      current,
      nominal,
      yaw,
      dt_s);

    publish_offboard_mode(timestamp);
    publish_setpoint(timestamp, command);

    if (auto_arm_ && tick_count_ >= 20 && (tick_count_ % 30 == 0)) {
      request_offboard_mode(timestamp);
      arm(timestamp);
    }

    publish_diagnostics(
      timestamp,
      command,
      odometry_stale,
      hard_override,
      target_authority,
      applied_authority);
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
    bool odometry_stale,
    bool hard_override,
    double target_authority,
    double applied_authority)
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
      kv("target_authority", std::to_string(target_authority)),
      kv("applied_authority", std::to_string(applied_authority)),
      kv("commanded_authority", std::to_string(supervisor_.commanded_authority())),
      kv("operation_context", to_string(supervisor_.operation_context())),
      kv("fault_label", fault_label_),
      kv("planner_mode", planner_mode_),
      kv("hold", command.hold ? "true" : "false"),
      kv("mission_paused", command.hold ? "true" : "false"),
      kv("velocity_limit_mps", std::to_string(command.velocity_limit)),
      kv("setpoint_x", std::to_string(command.position[0])),
      kv("setpoint_y", std::to_string(command.position[1])),
      kv("setpoint_z", std::to_string(command.position[2])),
      kv("odometry_stale", odometry_stale ? "true" : "false"),
      kv("hard_override_active", hard_override ? "true" : "false"),
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
  double min_authority_scale_{0.15};
  double trust_{1.0};
  double residual_{0.0};
  double authority_scale_{1.0};
  double target_authority_{1.0};
  double mission_elapsed_s_{0.0};
  bool has_odometry_{false};
  bool has_target_authority_{false};
  bool hard_override_active_{false};
  bool auto_arm_{false};
  bool force_arm_{false};
  bool offboard_requested_{false};
  bool arm_requested_{false};
  std::string fault_label_{"nominal"};
  std::string planner_mode_{"mission_fallback"};
  std::array<double, 3> nominal_setpoint_{0.0, 0.0, -2.0};
  std::array<double, 3> current_position_{0.0, 0.0, 0.0};
  CircleMissionParams mission_params_;
  std::vector<twinguard::planning::Obstacle> obstacles_;
  BT::Blackboard::Ptr bt_blackboard_;
  std::unique_ptr<BT::Tree> bt_tree_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
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
