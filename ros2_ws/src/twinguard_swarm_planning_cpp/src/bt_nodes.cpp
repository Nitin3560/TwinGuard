#include "twinguard_swarm_planning_cpp/bt_nodes.hpp"

#include "twinguard_swarm_planning_cpp/astar_planner.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

namespace twinguard::planning
{

namespace
{

using Point3 = std::array<double, 3>;

template<typename T>
bool get_from_blackboard(const BT::TreeNode & node, const std::string & key, T & value)
{
  if (!node.config().blackboard) {
    return false;
  }
  try {
    value = node.config().blackboard->get<T>(key);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

template<typename T>
void set_on_blackboard(const BT::TreeNode & node, const std::string & key, const T & value)
{
  if (node.config().blackboard) {
    node.config().blackboard->set(key, value);
  }
}

double yaw_to_target(const Point3 & from, const Point3 & to, double fallback)
{
  const double dx = to[0] - from[0];
  const double dy = to[1] - from[1];
  if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
    return fallback;
  }
  return std::atan2(dy, dx);
}

class IsSuspectedAttack : public BT::ConditionNode
{
public:
  IsSuspectedAttack(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::string fault;
    if (!get_from_blackboard(*this, "fault_label", fault)) {
      return BT::NodeStatus::FAILURE;
    }
    return fault == "suspected_attack" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class PublishHoldSetpoint : public BT::SyncActionNode
{
public:
  PublishHoldSetpoint(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    Point3 current;
    if (!get_from_blackboard(*this, "current_position", current)) {
      return BT::NodeStatus::FAILURE;
    }
    set_on_blackboard(*this, "computed_setpoint", current);
    set_on_blackboard(*this, "computed_yaw", 0.0);
    return BT::NodeStatus::SUCCESS;
  }
};

class NeedsReplan : public BT::ConditionNode
{
public:
  NeedsReplan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    Point3 current;
    Point3 nominal;
    std::vector<Obstacle> obstacles;
    if (!get_from_blackboard(*this, "current_position", current) ||
      !get_from_blackboard(*this, "mission_setpoint", nominal) ||
      !get_from_blackboard(*this, "obstacles", obstacles))
    {
      return BT::NodeStatus::FAILURE;
    }

    for (const auto & obstacle : obstacles) {
      if (AStarPlanner::path_intersects_obstacle(current, nominal, obstacle)) {
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::FAILURE;
  }
};

class ComputeAStarPath : public BT::SyncActionNode
{
public:
  ComputeAStarPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    Point3 current;
    Point3 nominal;
    std::vector<Obstacle> obstacles;
    if (!get_from_blackboard(*this, "current_position", current) ||
      !get_from_blackboard(*this, "mission_setpoint", nominal) ||
      !get_from_blackboard(*this, "obstacles", obstacles))
    {
      return BT::NodeStatus::FAILURE;
    }

    AStarPlanner planner;
    const auto path = planner.plan(current, nominal, obstacles);
    if (!path || path->empty()) {
      return BT::NodeStatus::FAILURE;
    }
    set_on_blackboard(*this, "planned_path", *path);
    return BT::NodeStatus::SUCCESS;
  }
};

class PublishPathSetpoint : public BT::SyncActionNode
{
public:
  PublishPathSetpoint(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    Point3 current;
    std::vector<Point3> path;
    double fallback_yaw{0.0};
    if (!get_from_blackboard(*this, "current_position", current) ||
      !get_from_blackboard(*this, "planned_path", path))
    {
      return BT::NodeStatus::FAILURE;
    }
    (void)get_from_blackboard(*this, "mission_yaw", fallback_yaw);

    const Point3 target = path.size() > 1 ? path[1] : path.front();
    set_on_blackboard(*this, "computed_setpoint", target);
    set_on_blackboard(*this, "computed_yaw", yaw_to_target(current, target, fallback_yaw));
    return BT::NodeStatus::SUCCESS;
  }
};

class ComputeMissionSetpoint : public BT::SyncActionNode
{
public:
  ComputeMissionSetpoint(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    Point3 mission;
    double yaw{0.0};
    if (!get_from_blackboard(*this, "mission_setpoint", mission)) {
      return BT::NodeStatus::FAILURE;
    }
    (void)get_from_blackboard(*this, "mission_yaw", yaw);
    set_on_blackboard(*this, "computed_setpoint", mission);
    set_on_blackboard(*this, "computed_yaw", yaw);
    return BT::NodeStatus::SUCCESS;
  }
};

class PublishMissionSetpoint : public BT::SyncActionNode
{
public:
  PublishMissionSetpoint(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    Point3 computed;
    return get_from_blackboard(*this, "computed_setpoint", computed) ?
      BT::NodeStatus::SUCCESS :
      BT::NodeStatus::FAILURE;
  }
};

class TrajectoryClear : public BT::ConditionNode
{
public:
  TrajectoryClear(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // TODO: Replace with cross-agent trajectory-intent checks once supervisors
    // publish planned path intents on a shared swarm topic.
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

void registerTwinGuardBtNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<IsSuspectedAttack>("IsSuspectedAttack");
  factory.registerNodeType<PublishHoldSetpoint>("PublishHoldSetpoint");
  factory.registerNodeType<NeedsReplan>("NeedsReplan");
  factory.registerNodeType<ComputeAStarPath>("ComputeAStarPath");
  factory.registerNodeType<PublishPathSetpoint>("PublishPathSetpoint");
  factory.registerNodeType<ComputeMissionSetpoint>("ComputeMissionSetpoint");
  factory.registerNodeType<PublishMissionSetpoint>("PublishMissionSetpoint");
  factory.registerNodeType<TrajectoryClear>("TrajectoryClear");
}

}  // namespace twinguard::planning
