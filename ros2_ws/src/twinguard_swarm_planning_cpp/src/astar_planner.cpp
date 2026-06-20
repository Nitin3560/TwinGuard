#include "twinguard_swarm_planning_cpp/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

namespace twinguard::planning
{

namespace
{

struct Cell
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const Cell & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct QueueItem
{
  Cell cell;
  double priority{0.0};

  bool operator>(const QueueItem & other) const
  {
    return priority > other.priority;
  }
};

struct SearchNode
{
  Cell parent;
  double g{std::numeric_limits<double>::infinity()};
  bool has_parent{false};
};

int64_t key(const Cell & cell)
{
  constexpr int64_t offset = 1LL << 20;
  return ((static_cast<int64_t>(cell.x) + offset) << 42) ^
    ((static_cast<int64_t>(cell.y) + offset) << 21) ^
    (static_cast<int64_t>(cell.z) + offset);
}

double distance(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double distance(const Cell & a, const Cell & b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Cell world_to_cell(
  const std::array<double, 3> & point,
  const std::array<double, 3> & origin,
  double cell_size)
{
  return Cell{
    static_cast<int>(std::llround((point[0] - origin[0]) / cell_size)),
    static_cast<int>(std::llround((point[1] - origin[1]) / cell_size)),
    static_cast<int>(std::llround((point[2] - origin[2]) / cell_size)),
  };
}

std::array<double, 3> cell_to_world(
  const Cell & cell,
  const std::array<double, 3> & origin,
  double cell_size)
{
  return {
    origin[0] + static_cast<double>(cell.x) * cell_size,
    origin[1] + static_cast<double>(cell.y) * cell_size,
    origin[2] + static_cast<double>(cell.z) * cell_size,
  };
}

bool within_extent(const Cell & cell, int extent)
{
  return std::abs(cell.x) <= extent &&
    std::abs(cell.y) <= extent &&
    std::abs(cell.z) <= extent;
}

bool blocked(
  const Cell & cell,
  const std::array<double, 3> & origin,
  double cell_size,
  const std::vector<Obstacle> & obstacles)
{
  const auto world = cell_to_world(cell, origin, cell_size);
  return std::any_of(obstacles.begin(), obstacles.end(), [&](const Obstacle & obstacle) {
    return distance(world, obstacle.center) <= obstacle.radius_m;
  });
}

std::vector<std::array<double, 3>> reconstruct_path(
  const Cell & goal,
  const std::unordered_map<int64_t, SearchNode> & nodes,
  const std::array<double, 3> & origin,
  double cell_size)
{
  std::vector<std::array<double, 3>> path;
  Cell cursor = goal;
  while (true) {
    path.push_back(cell_to_world(cursor, origin, cell_size));
    const auto it = nodes.find(key(cursor));
    if (it == nodes.end() || !it->second.has_parent) {
      break;
    }
    cursor = it->second.parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace

AStarPlanner::AStarPlanner(GridConfig config)
: config_(config)
{
  config_.cell_size_m = std::max(config_.cell_size_m, 0.05);
  config_.grid_extent_cells = std::max(config_.grid_extent_cells, 2);
}

std::optional<std::vector<std::array<double, 3>>> AStarPlanner::plan(
  const std::array<double, 3> & start,
  const std::array<double, 3> & goal,
  const std::vector<Obstacle> & obstacles) const
{
  const std::array<double, 3> origin{
    0.5 * (start[0] + goal[0]),
    0.5 * (start[1] + goal[1]),
    0.5 * (start[2] + goal[2]),
  };
  const Cell start_cell = world_to_cell(start, origin, config_.cell_size_m);
  const Cell goal_cell = world_to_cell(goal, origin, config_.cell_size_m);

  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  std::unordered_map<int64_t, SearchNode> nodes;

  nodes[key(start_cell)] = SearchNode{Cell{}, 0.0, false};
  open.push(QueueItem{start_cell, distance(start_cell, goal_cell)});

  while (!open.empty()) {
    const Cell current = open.top().cell;
    open.pop();

    if (current == goal_cell) {
      return reconstruct_path(goal_cell, nodes, origin, config_.cell_size_m);
    }

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const Cell next{current.x + dx, current.y + dy, current.z + dz};
          if (!within_extent(next, config_.grid_extent_cells) ||
            blocked(next, origin, config_.cell_size_m, obstacles))
          {
            continue;
          }

          const auto current_it = nodes.find(key(current));
          if (current_it == nodes.end()) {
            continue;
          }
          const double tentative_g = current_it->second.g + distance(current, next);
          auto & next_node = nodes[key(next)];
          if (tentative_g < next_node.g) {
            next_node.g = tentative_g;
            next_node.parent = current;
            next_node.has_parent = true;
            open.push(QueueItem{next, tentative_g + distance(next, goal_cell)});
          }
        }
      }
    }
  }

  return std::nullopt;
}

bool AStarPlanner::path_intersects_obstacle(
  const std::array<double, 3> & start,
  const std::array<double, 3> & goal,
  const Obstacle & obstacle)
{
  const std::array<double, 3> segment{
    goal[0] - start[0],
    goal[1] - start[1],
    goal[2] - start[2],
  };
  const double length_sq =
    segment[0] * segment[0] + segment[1] * segment[1] + segment[2] * segment[2];
  if (length_sq <= 1e-9) {
    return distance(start, obstacle.center) <= obstacle.radius_m;
  }

  const std::array<double, 3> start_to_center{
    obstacle.center[0] - start[0],
    obstacle.center[1] - start[1],
    obstacle.center[2] - start[2],
  };
  const double t = std::clamp(
    (start_to_center[0] * segment[0] +
    start_to_center[1] * segment[1] +
    start_to_center[2] * segment[2]) / length_sq,
    0.0,
    1.0);
  const std::array<double, 3> closest{
    start[0] + t * segment[0],
    start[1] + t * segment[1],
    start[2] + t * segment[2],
  };
  return distance(closest, obstacle.center) <= obstacle.radius_m;
}

}  // namespace twinguard::planning
