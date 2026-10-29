#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <tuple>
#include <vector>

namespace grid_visualizer_pkg {

struct RecoveryPoint {double x{0}, y{0};};
inline double recovery_distance(RecoveryPoint a, RecoveryPoint b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

struct RecoveryGrid {
  int rows{0}, cols{0};
  double cell_size{0.5};
  RecoveryPoint center;
  std::vector<std::uint8_t> blocked;

  bool valid() const
  {
    return rows > 2 && cols > 2 && cell_size > 0 &&
           blocked.size() == static_cast<std::size_t>(rows * cols);
  }
  bool free(int row, int col) const
  {
    return row > 0 && col > 0 && row < rows - 1 && col < cols - 1 &&
           !blocked[row * cols + col];
  }
  int index(RecoveryPoint p) const
  {
    const double col = (p.x - center.x) / cell_size + cols / 2.0;
    const double row = (center.y - p.y) / cell_size + rows / 2.0;
    if (!std::isfinite(row) || !std::isfinite(col) ||
      row < 0 || col < 0 || row >= rows || col >= cols) {return -1;}
    return static_cast<int>(std::floor(row)) * cols + static_cast<int>(std::floor(col));
  }
  RecoveryPoint position(int index) const
  {
    return {center.x + (index % cols + 0.5 - cols / 2.0) * cell_size,
      center.y - (index / cols + 0.5 - rows / 2.0) * cell_size};
  }
  bool edge(int row, int col, int dr, int dc) const
  {
    return free(row + dr, col + dc) &&
           (!(dr && dc) || (free(row + dr, col) && free(row, col + dc)));
  }
};

// Shared by ordinary low-gradient fallback and retained-target recovery.
inline std::vector<double> recovery_distances(
  const RecoveryGrid & grid, const std::vector<std::pair<int, double>> & sources,
  double epsilon = 1e-10)
{
  std::vector<double> distances(grid.blocked.size(), std::numeric_limits<double>::infinity());
  if (!grid.valid()) {return distances;}
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  for (auto [index, cost] : sources) {
    if (index < 0 || index >= grid.rows * grid.cols ||
      !grid.free(index / grid.cols, index % grid.cols) || !std::isfinite(cost)) {continue;}
    if (cost < distances[index]) {distances[index] = cost; queue.emplace(cost, index);}
  }
  while (!queue.empty()) {
    const auto [cost, index] = queue.top();
    queue.pop();
    if (cost > distances[index]) {continue;}
    const int row = index / grid.cols, col = index % grid.cols;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if ((!dr && !dc) || !grid.edge(row, col, dr, dc)) {continue;}
        const int next = (row + dr) * grid.cols + col + dc;
        const double candidate = cost + grid.cell_size * std::hypot(dr, dc);
        if (candidate + epsilon < distances[next]) {
          distances[next] = candidate;
          queue.emplace(candidate, next);
        }
      }
    }
  }
  return distances;
}

// Exact grid traversal for commanded segments, including diagonal corner ties.
inline bool recovery_segment_free(const RecoveryGrid & grid, RecoveryPoint from, RecoveryPoint to)
{
  if (!grid.valid()) {return false;}
  const int first=grid.index(from),last=grid.index(to);
  if (first<0 || last<0 || !grid.free(first/grid.cols,first%grid.cols)) {return false;}
  int row=first/grid.cols,col=first%grid.cols;
  const double x=(from.x-grid.center.x)/grid.cell_size+grid.cols/2.0;
  const double y=(grid.center.y-from.y)/grid.cell_size+grid.rows/2.0;
  const double dx=(to.x-from.x)/grid.cell_size,dy=(from.y-to.y)/grid.cell_size;
  const int dc=(dx>0)-(dx<0),dr=(dy>0)-(dy<0);
  const double infinity=std::numeric_limits<double>::infinity();
  double tx=dc ? ((dc>0 ? col+1.0:col)-x)/dx:infinity;
  double ty=dr ? ((dr>0 ? row+1.0:row)-y)/dy:infinity;
  while (row*grid.cols+col!=last) {
    const bool both=std::abs(tx-ty)<1e-12;
    const int step_col=(tx<ty || both) ? dc:0;
    const int step_row=(ty<tx || both) ? dr:0;
    if ((!step_col && !step_row) || !grid.edge(row,col,step_row,step_col)) {return false;}
    row+=step_row;col+=step_col;
    if (step_col) {tx+=1/std::abs(dx);}
    if (step_row) {ty+=1/std::abs(dy);}
  }
  return true;
}

struct RecoveryTarget {
  RecoveryPoint position;  // Real global destination, never its clipped proxy.
  RecoveryPoint identity;  // Cow position for spatial association.
  bool search{false};
  int patch_id{-1};
  int cell{-1};            // This snapshot's placed local waypoint.
  double strength{0}, source_cost{0};
};

struct RecoverySettings {
  double release_m{1}, agreement_s{3}, alignment_deg{45}, window_progress_m{0.3};
  double watch_s{6}, backtrack_m{1}, opposition_s{1}, arrival_m{0.5};
  double association_m{2}, max_step_m{0.5}, epsilon{1e-10};
};

enum class RecoveryPhase {idle, recovering, watching};
enum class RecoveryEvent {entered, retargeted, invalidated, handed_off, reactivated,
  arrived, watch_ended, clock_reset};

inline const char * recovery_event_name(RecoveryEvent event)
{
  switch (event) {
    case RecoveryEvent::entered: return "entered";
    case RecoveryEvent::retargeted: return "retargeted";
    case RecoveryEvent::invalidated: return "invalidated";
    case RecoveryEvent::handed_off: return "handed_off";
    case RecoveryEvent::reactivated: return "reactivated";
    case RecoveryEvent::arrived: return "arrived";
    case RecoveryEvent::watch_ended: return "watch_ended";
    case RecoveryEvent::clock_reset: return "clock_reset";
  }
  return "unknown";
}

struct RecoveryResult {
  bool active{false}, has_step{false};
  RecoveryPoint movement;
  const char * reason{nullptr};
  std::vector<RecoveryEvent> events;
};

class DistanceRecovery {
public:
  RecoveryPhase phase() const {return phase_;}
  const std::optional<RecoveryTarget> & target() const {return target_;}
  void reset()
  {
    phase_ = RecoveryPhase::idle;
    target_.reset();
    qualification_.clear();
    opposition_since_.reset();
    last_time_.reset();
  }

  RecoveryResult update(
    double now, RecoveryPoint position, const RecoveryGrid & grid,
    const std::vector<RecoveryTarget> & candidates, RecoveryPoint normal,
    bool converged, bool requested, const RecoverySettings & settings)
  {
    RecoveryResult result;
    if (last_time_ && now < *last_time_) {
      reset();
      result.events.push_back(RecoveryEvent::clock_reset);
    }
    last_time_ = now;
    if (phase_ == RecoveryPhase::idle && !requested) {return result;}
    if (phase_ == RecoveryPhase::idle) {
      phase_ = RecoveryPhase::recovering;
      result.events.push_back(RecoveryEvent::entered);
    }
    result.active = phase_ == RecoveryPhase::recovering;
    const int start = grid.valid() ? grid.index(position) : -1;
    if (start < 0 || !grid.free(start / grid.cols, start % grid.cols)) {
      qualification_.clear();
      return result;  // No safe current cell: never manufacture an escape vector.
    }
    const auto reachable = recovery_distances(grid, {{start, 0}}, settings.epsilon);
    auto usable = [&](const RecoveryTarget & candidate) {
        return candidate.strength > 0 && candidate.cell >= 0 &&
               candidate.cell < static_cast<int>(reachable.size()) &&
               std::isfinite(reachable[candidate.cell]) &&
               std::isfinite(candidate.position.x) && std::isfinite(candidate.position.y) &&
               (candidate.cell != start || grid.index(candidate.position) == start ||
               recovery_distance(position, candidate.position) <= settings.arrival_m);
      };
    const RecoveryTarget * matched = nullptr;
    double nearest = std::numeric_limits<double>::infinity();
    if (target_) {
      for (const auto & candidate : candidates) {
        if (!usable(candidate) || candidate.search != target_->search) {continue;}
        const double association = candidate.search ?
          (candidate.patch_id == target_->patch_id ? 0 : std::numeric_limits<double>::infinity()) :
          recovery_distance(candidate.identity, target_->identity);
        if (association <= settings.association_m &&
          (association < nearest || (association == nearest && matched && ordered(candidate, *matched))))
        {
          matched = &candidate;
          nearest = association;
        }
      }
      if (!matched) {
        result.events.push_back(RecoveryEvent::invalidated);
        target_.reset();
        qualification_.clear();
        if (phase_ == RecoveryPhase::watching) {
          phase_ = requested ? RecoveryPhase::recovering : RecoveryPhase::idle;
        }
      }
    }
    if (matched) {
      target_ = *matched;
      if (recovery_distance(target_->position, reference_) > grid.cell_size) {anchor(position);}
    } else if (phase_ == RecoveryPhase::recovering) {
      const RecoveryTarget * best = nullptr;
      double best_score = std::numeric_limits<double>::infinity();
      for (const auto & candidate : candidates) {
        if (!usable(candidate) || !std::isfinite(candidate.source_cost)) {continue;}
        const double score = reachable[candidate.cell] + candidate.source_cost;
        if (score + settings.epsilon < best_score ||
          (std::abs(score - best_score) <= settings.epsilon && (!best || ordered(candidate, *best))))
        {
          best = &candidate;
          best_score = score;
        }
      }
      if (best) {
        target_ = *best;
        anchor(position);
        result.events.push_back(RecoveryEvent::retargeted);
      }
    }
    result.active = phase_ == RecoveryPhase::recovering;
    if (!target_) {return result;}
    if (recovery_distance(position, target_->position) <= settings.arrival_m) {
      result.events.push_back(RecoveryEvent::arrived);
      reset();
      result.active = false;
      return result;
    }

    const auto distances = recovery_distances(grid, {{target_->cell, 0}}, settings.epsilon);
    int next = -1;
    double best = distances[start];
    const int row = start / grid.cols, col = start % grid.cols;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if ((!dr && !dc) || !grid.edge(row, col, dr, dc)) {continue;}
        const int index = (row + dr) * grid.cols + col + dc;
        if (distances[index] + settings.epsilon < best) {best = distances[index]; next = index;}
      }
    }
    RecoveryPoint waypoint;
    if (next >= 0) {
      waypoint = grid.position(next);
    } else if (start == target_->cell && grid.index(target_->position) == start) {
      waypoint = target_->position;
    } else {
      qualification_.clear();
      return result;
    }
    const double length = recovery_distance(position, waypoint);
    if (length <= settings.epsilon) {return result;}
    const RecoveryPoint direction{(waypoint.x - position.x) / length, (waypoint.y - position.y) / length};
    const double step = std::min({length, grid.cell_size, settings.max_step_m});
    result.movement = {direction.x * step, direction.y * step};
    if (!recovery_segment_free(grid,position,
        {position.x+result.movement.x,position.y+result.movement.y})) {
      qualification_.clear();
      return result;
    }
    result.has_step = true;
    const double normal_length = std::hypot(normal.x, normal.y);
    const double dot = normal_length > settings.epsilon ?
      (normal.x * direction.x + normal.y * direction.y) / normal_length : -1;
    const double distance = recovery_distance(position, reference_);

    if (phase_ == RecoveryPhase::watching) {
      best_distance_ = std::min(best_distance_, distance);
      if (!converged || dot < 0) {
        if (!opposition_since_) {opposition_since_ = now;}
      } else {opposition_since_.reset();}
      if (requested || distance - best_distance_ >= settings.backtrack_m ||
        (opposition_since_ && now - *opposition_since_ >= settings.opposition_s))
      {
        result.reason=requested ? "requested" :
          (distance-best_distance_>=settings.backtrack_m ? "backtrack" : "opposition");
        phase_ = RecoveryPhase::recovering;
        anchor(position);
        result.events.push_back(RecoveryEvent::reactivated);
      } else if (now - watch_started_ >= settings.watch_s) {
        result.events.push_back(RecoveryEvent::watch_ended);
        reset();
      }
    } else {
      const double cosine = std::cos(settings.alignment_deg * std::acos(-1.0) / 180);
      if (!converged || dot < cosine ||
        (!qualification_.empty() && distance > qualification_.back().distance + 0.05))
      {
        qualification_.clear();
      } else {
        if (qualification_.empty() || now > qualification_.back().time) {
          qualification_.push_back({now, distance});
        }
        while (qualification_.size() > 1 && qualification_[1].time <= now - settings.agreement_s) {
          qualification_.pop_front();
        }
        if (now - qualification_.front().time >= settings.agreement_s &&
          initial_distance_ - distance >= settings.release_m &&
          qualification_.front().distance - distance >= settings.window_progress_m)
        {
          phase_ = RecoveryPhase::watching;
          watch_started_ = now;
          best_distance_ = distance;
          opposition_since_.reset();
          qualification_.clear();
          result.events.push_back(RecoveryEvent::handed_off);
        }
      }
    }
    result.active = phase_ == RecoveryPhase::recovering;
    return result;
  }

private:
  static bool ordered(const RecoveryTarget & a, const RecoveryTarget & b)
  {
    return std::tie(a.search, a.position.x, a.position.y, a.patch_id) <
           std::tie(b.search, b.position.x, b.position.y, b.patch_id);
  }
  void anchor(RecoveryPoint position)
  {
    reference_ = target_->position;
    initial_distance_ = recovery_distance(position, reference_);
    best_distance_ = initial_distance_;
    qualification_.clear();
    opposition_since_.reset();
  }
  struct Qualification {double time, distance;};
  RecoveryPhase phase_{RecoveryPhase::idle};
  std::optional<RecoveryTarget> target_;
  RecoveryPoint reference_;
  double initial_distance_{0}, best_distance_{0}, watch_started_{0};
  std::optional<double> last_time_, opposition_since_;
  std::deque<Qualification> qualification_;
};
}  // namespace grid_visualizer_pkg
