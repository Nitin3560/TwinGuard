#pragma once

#include <array>
#include <optional>
#include <vector>

namespace twinguard::planning
{

struct Obstacle
{
  std::array<double, 3> center{0.0, 0.0, 0.0};
  double radius_m{1.0};
};

struct GridConfig
{
  double cell_size_m{0.5};
  // Local grid extent around the start/goal midpoint.
  int grid_extent_cells{32};
};

class AStarPlanner
{
public:
  explicit AStarPlanner(GridConfig config = GridConfig{});

  std::optional<std::vector<std::array<double, 3>>> plan(
    const std::array<double, 3> & start,
    const std::array<double, 3> & goal,
    const std::vector<Obstacle> & obstacles) const;

  static bool path_intersects_obstacle(
    const std::array<double, 3> & start,
    const std::array<double, 3> & goal,
    const Obstacle & obstacle);

private:
  GridConfig config_;
};

}  // namespace twinguard::planning
