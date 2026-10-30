#ifndef GRID_VISUALIZER_PKG__GOAL_QUIET_ZONE_HPP_
#define GRID_VISUALIZER_PKG__GOAL_QUIET_ZONE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace grid_visualizer_pkg
{

struct CowPoint
{
  double x;
  double y;
};

inline bool inside_cow_exclusion(
  CowPoint point, const std::vector<CowPoint> & cows, double radius)
{
  return std::any_of(cows.begin(), cows.end(), [&](const CowPoint & cow) {
    return std::hypot(point.x - cow.x, point.y - cow.y) <= radius;
  });
}

inline std::size_t closest_cow_outside_goal(
  const std::vector<CowPoint> & cows, CowPoint drone, CowPoint goal,
  double goal_radius)
{
  std::size_t closest = cows.size();
  double closest_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < cows.size(); ++index) {
    if (std::hypot(cows[index].x - goal.x, cows[index].y - goal.y) <=
      goal_radius)
    {
      continue;
    }
    const double distance = std::hypot(
      cows[index].x - drone.x, cows[index].y - drone.y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest = index;
    }
  }
  return closest;
}

class GoalQuietZone
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  GoalQuietZone(
    CowPoint goal, double goal_radius, double quiet_radius,
    double quiet_duration_seconds)
  : goal_(goal), goal_radius_(goal_radius), quiet_radius_(quiet_radius),
    quiet_duration_(std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(quiet_duration_seconds)))
  {}

  double update(const std::vector<CowPoint> & cows, TimePoint now)
  {
    struct Candidate
    {
      std::size_t old_index;
      std::size_t new_index;
      double distance;
    };

    std::vector<Candidate> candidates;
    for (std::size_t old_index = 0; old_index < previous_cows_.size(); ++old_index) {
      for (std::size_t new_index = 0; new_index < cows.size(); ++new_index) {
        const double distance = std::hypot(
          previous_cows_[old_index].position.x - cows[new_index].x,
          previous_cows_[old_index].position.y - cows[new_index].y);
        if (distance <= quiet_radius_) {
          candidates.push_back({old_index, new_index, distance});
        }
      }
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & lhs, const Candidate & rhs) {
        return lhs.distance < rhs.distance;
      });

    std::vector<int> old_for_new(cows.size(), -1);
    std::vector<bool> old_used(previous_cows_.size(), false);
    for (const auto & candidate : candidates) {
      if (!old_used[candidate.old_index] && old_for_new[candidate.new_index] < 0) {
        old_for_new[candidate.new_index] = static_cast<int>(candidate.old_index);
        old_used[candidate.old_index] = true;
      }
    }

    bool new_arrival = false;
    std::vector<RememberedCow> next_cows;
    next_cows.reserve(cows.size());
    for (std::size_t index = 0; index < cows.size(); ++index) {
      const double goal_distance = std::hypot(
        cows[index].x - goal_.x, cows[index].y - goal_.y);
      bool arrival_latched = old_for_new[index] >= 0 &&
        previous_cows_[old_for_new[index]].arrival_latched;

      // A cow must leave the entire quiet zone before another entry can
      // restart the timer. Readings that wobble across the normal goal radius cannot extend it.
      if (goal_distance > quiet_radius_) {
        arrival_latched = false;
      } else if (goal_distance <= goal_radius_ && !arrival_latched) {
        new_arrival = true;
        arrival_latched = true;
      }
      next_cows.push_back({cows[index], arrival_latched});
    }
    previous_cows_ = std::move(next_cows);

    if (new_arrival) {
      quiet_until_ = now + quiet_duration_;
    }
    return now < quiet_until_ ? quiet_radius_ : goal_radius_;
  }

private:
  struct RememberedCow
  {
    CowPoint position;
    bool arrival_latched;
  };

  CowPoint goal_;
  double goal_radius_;
  double quiet_radius_;
  Clock::duration quiet_duration_;
  TimePoint quiet_until_{};
  std::vector<RememberedCow> previous_cows_;
};

}  // namespace grid_visualizer_pkg

#endif  // GRID_VISUALIZER_PKG__GOAL_QUIET_ZONE_HPP_
