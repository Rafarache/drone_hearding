#include <chrono>
#include <cassert>
#include <stdexcept>
#include <vector>

#include "grid_visualizer_pkg/goal_quiet_zone.hpp"

using grid_visualizer_pkg::CowPoint;
using grid_visualizer_pkg::GoalQuietZone;
using grid_visualizer_pkg::closest_cow_outside_goal;

void expect_radius(double actual, double expected)
{
  if (actual != expected) {
    throw std::runtime_error("Unexpected effective goal radius");
  }
}

int main()
{
  // Cow A's pushing point is (8,0). Cow B suppresses it on the closed
  // exclusion boundary, regardless of input order; moving B clear restores it.
  using grid_visualizer_pkg::inside_cow_exclusion;
  assert(inside_cow_exclusion({8,0}, {{4,0},{8,0}}, 2));
  assert(inside_cow_exclusion({8,0}, {{4,0},{10,0}}, 2));
  assert(inside_cow_exclusion({8,0}, {{10,0},{4,0}}, 2));
  assert(!inside_cow_exclusion({8,0}, {{4,0},{10.001,0}}, 2));
  assert(!inside_cow_exclusion({8,0}, {{4,0}}, 2));
  assert(!inside_cow_exclusion({8,0}, {}, 2));

  const GoalQuietZone::TimePoint arrival{};
  GoalQuietZone separate_radii({0,0}, 3.0, 5.0, 20.0);
  expect_radius(separate_radii.update({{4,0}}, arrival), 3.0);
  expect_radius(separate_radii.update({{3,0},{4,0}}, arrival), 5.0);
  expect_radius(separate_radii.update({{3,0},{4,0}}, arrival + std::chrono::seconds(19)), 5.0);
  expect_radius(separate_radii.update({{3,0},{4,0}}, arrival + std::chrono::seconds(20)), 3.0);

  const GoalQuietZone::TimePoint start{};
  GoalQuietZone zone({0.0, 0.0}, 2.0, 4.0, 5.0);

  expect_radius(zone.update({}, start), 2.0);
  expect_radius(zone.update({{6.0, 0.0}}, start), 2.0);
  expect_radius(zone.update({{1.5, 0.0}, {6.0, 0.0}},
    start + std::chrono::seconds(1)), 4.0);
  // The other cow remains outside the effective goal radius.
  if (!(6.0 > zone.update({{2.5, 0.0}, {6.0, 0.0}},
      start + std::chrono::seconds(2))))
  {
    throw std::runtime_error("A cow outside 4 m lost its push objective");
  }
  expect_radius(zone.update({{1.8, 0.0}, {6.0, 0.0}},
    start + std::chrono::seconds(3)), 4.0);
  expect_radius(zone.update({{2.5, 0.0}, {6.0, 0.0}},
    start + std::chrono::seconds(6)), 2.0);
  expect_radius(zone.update({{1.8, 0.0}, {6.0, 0.0}},
    start + std::chrono::seconds(7)), 2.0);

  GoalQuietZone another_arrival({0.0, 0.0}, 2.0, 4.0, 5.0);
  expect_radius(another_arrival.update({{1.0, 0.0}}, start), 4.0);
  expect_radius(another_arrival.update({{1.0, 0.0}, {1.5, 0.0}},
    start + std::chrono::seconds(2)), 4.0);
  expect_radius(another_arrival.update({{1.0, 0.0}, {1.5, 0.0}},
    start + std::chrono::seconds(5)), 4.0);
  expect_radius(another_arrival.update({{1.0, 0.0}, {1.5, 0.0}},
    start + std::chrono::seconds(7)), 2.0);

  GoalQuietZone reentry({0.0, 0.0}, 2.0, 4.0, 5.0);
  expect_radius(reentry.update({{1.0, 0.0}}, start), 4.0);
  expect_radius(reentry.update({{5.0, 0.0}},
    start + std::chrono::seconds(1)), 4.0);
  expect_radius(reentry.update({{1.0, 0.0}},
    start + std::chrono::seconds(2)), 4.0);
  expect_radius(reentry.update({{1.0, 0.0}},
    start + std::chrono::seconds(6)), 4.0);
  expect_radius(reentry.update({{1.0, 0.0}},
    start + std::chrono::seconds(7)), 2.0);

  const std::vector<CowPoint> cows{
    {1.0, 0.0}, {3.0, 0.0}, {5.0, 0.0}, {6.0, 0.0}};
  const CowPoint drone{2.0, 0.0};
  const CowPoint goal{0.0, 0.0};
  if (closest_cow_outside_goal(cows, drone, goal, 4.0) != 2) {
    throw std::runtime_error("Focus did not choose the nearest cow outside 4 m");
  }
  if (closest_cow_outside_goal(cows, drone, goal, 2.0) != 1) {
    throw std::runtime_error("Focus did not use the restored 2 m radius");
  }
  if (closest_cow_outside_goal({{1.0, 0.0}}, drone, goal, 4.0) != 1) {
    throw std::runtime_error("Focus selected a cow inside the goal radius");
  }

  return 0;
}
