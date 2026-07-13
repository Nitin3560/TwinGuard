#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "twinguard_swarm_planning_cpp/astar_planner.hpp"

using twinguard::planning::AStarPlanner;
using twinguard::planning::GridConfig;
using twinguard::planning::Obstacle;

namespace
{

double distance(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

TEST(AStarPlanner, FindsStraightPathWithoutObstacles)
{
  AStarPlanner planner(GridConfig{0.5, 16});
  const std::array<double, 3> start{0.0, 0.0, 0.0};
  const std::array<double, 3> goal{2.0, 0.0, 0.0};

  const auto path = planner.plan(start, goal, {});

  ASSERT_TRUE(path.has_value());
  ASSERT_GE(path->size(), 2u);
  EXPECT_NEAR(distance(path->front(), start), 0.0, 1e-9);
  EXPECT_NEAR(distance(path->back(), goal), 0.0, 1e-9);
}

TEST(AStarPlanner, ReroutesAroundSingleObstacle)
{
  AStarPlanner planner(GridConfig{0.5, 16});
  const std::array<double, 3> start{-2.0, 0.0, 0.0};
  const std::array<double, 3> goal{2.0, 0.0, 0.0};
  const Obstacle obstacle{{0.0, 0.0, 0.0}, 0.6};

  ASSERT_TRUE(AStarPlanner::path_intersects_obstacle(start, goal, obstacle));
  const auto path = planner.plan(start, goal, {obstacle});

  ASSERT_TRUE(path.has_value());
  bool leaves_centerline = false;
  for (const auto & point : *path) {
    EXPECT_GT(distance(point, obstacle.center), obstacle.radius_m);
    leaves_centerline = leaves_centerline || std::abs(point[1]) > 0.1 || std::abs(point[2]) > 0.1;
  }
  EXPECT_TRUE(leaves_centerline);
}

TEST(AStarPlanner, ReturnsNulloptWhenLocalGridIsBlocked)
{
  AStarPlanner planner(GridConfig{0.5, 4});
  const std::array<double, 3> start{0.0, 0.0, 0.0};
  const std::array<double, 3> goal{1.0, 0.0, 0.0};
  const Obstacle obstacle{{0.5, 0.0, 0.0}, 5.0};

  const auto path = planner.plan(start, goal, {obstacle});

  EXPECT_FALSE(path.has_value());
}
