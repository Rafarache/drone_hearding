#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace grid_visualizer_pkg
{
struct EvidenceDetection
{
  double x{0}, y{0};
  bool position_valid{false};
  double bearing_min{0}, bearing_max{0};
};

struct EvidenceFrame
{
  int source{0};
  int64_t stamp_ns{0};
  double x{0}, y{0}, yaw{0};
  double near{2}, far{20}, fov{2.09};
  std::vector<EvidenceDetection> detections;
};

// Per-cell evidence only: no identities, association, prediction or timer decay.
class CowEvidence
{
public:
  CowEvidence(double min_x, double min_y, double width, double height, double cell_size,
    int own_source, int sources)
  : min_x_(min_x), min_y_(min_y), cell_(cell_size), own_(own_source), sources_(sources)
  {
    if (!std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(width) ||
      !std::isfinite(height) || !std::isfinite(cell_size) || width <= 0 || height <= 0 ||
      cell_size <= 0 || own_source < 0 || own_source >= sources)
    {throw std::invalid_argument("Invalid cow evidence grid");}
    cols_ = static_cast<int>(std::llround(width / cell_));
    rows_ = static_cast<int>(std::llround(height / cell_));
    if (cols_ < 1 || rows_ < 1 || std::abs(cols_*cell_-width) > 1e-8 ||
      std::abs(rows_*cell_-height) > 1e-8)
    {throw std::invalid_argument("Cow evidence dimensions must divide into whole cells");}
    scores_.resize(rows_*cols_, 0);
  }

  void advance_time(int64_t now_ns)
  {
    if (last_now_ && now_ns < *last_now_) {
      std::fill(scores_.begin(), scores_.end(), 0);
      pending_.clear();
      last_stamp_.clear();
    }
    last_now_ = now_ns;
  }

  bool enqueue(EvidenceFrame frame)
  {
    if (frame.source < 0 || frame.source >= sources_ || !valid(frame)) {return false;}
    const auto previous = last_stamp_.find(frame.source);
    if (previous != last_stamp_.end() && frame.stamp_ns <= previous->second) {return false;}
    last_stamp_[frame.source] = frame.stamp_ns;
    pending_.push_back(std::move(frame));
    return true;
  }

  void update()
  {
    // Gather protection before scoring, so callback arrival order within this
    // update cannot let an own-camera miss erase a peer-camera hit.
    std::vector<bool> protected_cells(scores_.size(), false);
    std::vector<std::vector<int>> hits;
    for (const auto& frame : pending_) {
      hits.push_back(hit_cells(frame));
      for (int i : hits.back()) {protected_cells[i] = true;}
    }
    for (std::size_t f=0; f<pending_.size(); ++f) {
      if (pending_[f].source == own_) {
        for (int i : hits[f]) {adjust(i, 150);}
      }
    }
    for (const auto& frame : pending_) {
      if (frame.source != own_) {continue;}
      for (int i=0; i<static_cast<int>(scores_.size()); ++i) {
        if (!protected_cells[i] && visible_empty(i, frame)) {adjust(i, -30);}
      }
    }
    for (std::size_t f=0; f<pending_.size(); ++f) {
      if (pending_[f].source != own_) {
        for (int i : hits[f]) {adjust(i, 150);}
      }
    }
    // Peer protection and positive evidence are consumed, never cached/replayed.
    pending_.clear();
  }

  static bool is_cow(int confidence) {return confidence > 180;}

  int score(double x, double y) const
  {
    const int i = index(x,y);
    return i < 0 ? 0 : scores_[i];
  }

  std::vector<std::pair<double,double>> active_positions() const
  {
    std::vector<std::pair<double,double>> result;
    for (int i=0; i<static_cast<int>(scores_.size()); ++i) {
      if (is_cow(scores_[i])) {result.push_back(center(i));}
    }
    return result;
  }

private:
  static constexpr double pi = 3.14159265358979323846;
  double min_x_, min_y_, cell_;
  int cols_, rows_, own_, sources_;
  std::vector<uint8_t> scores_;
  std::vector<EvidenceFrame> pending_;
  std::map<int,int64_t> last_stamp_;
  std::optional<int64_t> last_now_;

  static bool valid(const EvidenceFrame& f)
  {
    if (f.stamp_ns < 0 || !std::isfinite(f.x) || !std::isfinite(f.y) ||
      !std::isfinite(f.yaw) || !std::isfinite(f.near) || !std::isfinite(f.far) ||
      !std::isfinite(f.fov) || f.near < 0 || f.far <= f.near || f.fov <= 0 || f.fov > 2*pi)
    {return false;}
    for (const auto& d : f.detections) {
      if (!std::isfinite(d.bearing_min) || !std::isfinite(d.bearing_max) ||
        d.bearing_min < -pi || d.bearing_max > pi || d.bearing_min > d.bearing_max ||
        (d.position_valid && (!std::isfinite(d.x) || !std::isfinite(d.y))))
      {return false;}
    }
    return true;
  }

  int index(double x, double y) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || x < min_x_ || y < min_y_ ||
      x >= min_x_+cols_*cell_ || y >= min_y_+rows_*cell_) {return -1;}
    return static_cast<int>((y-min_y_)/cell_)*cols_ + static_cast<int>((x-min_x_)/cell_);
  }

  std::pair<double,double> center(int i) const
  {
    return {min_x_+(i%cols_+0.5)*cell_, min_y_+(i/cols_+0.5)*cell_};
  }

  std::vector<int> hit_cells(const EvidenceFrame& f) const
  {
    std::vector<int> result;
    for (const auto& d : f.detections) {
      if (!d.position_valid || std::hypot(d.x-f.x,d.y-f.y) > f.far) {continue;}
      const int i = index(d.x,d.y);
      if (i >= 0) {result.push_back(i);}
    }
    std::sort(result.begin(),result.end());
    result.erase(std::unique(result.begin(),result.end()),result.end());
    return result;
  }

  bool visible_empty(int i, const EvidenceFrame& f) const
  {
    const auto p = center(i);
    const double dx=p.first-f.x, dy=p.second-f.y, range=std::hypot(dx,dy);
    // The circumscribed disk conservatively covers the entire square cell.
    const double radius=cell_*std::sqrt(0.5);
    if (range-radius < f.near || range+radius > f.far || range <= radius) {return false;}
    const double bearing=std::remainder(std::atan2(dy,dx)-f.yaw,2*pi);
    const double angle_radius=std::asin(radius/range);
    if (std::abs(bearing)+angle_radius > f.fov/2) {return false;}
    for (const auto& d : f.detections) {
      const double shadow_start=d.position_valid ? std::max(0.0,std::hypot(d.x-f.x,d.y-f.y)-1.0) : 0.0;
      if (range+radius >= shadow_start && bearing+angle_radius >= d.bearing_min &&
        bearing-angle_radius <= d.bearing_max) {return false;}
    }
    return true;
  }

  void adjust(int i, int delta)
  {
    scores_[i] = static_cast<uint8_t>(std::clamp(static_cast<int>(scores_[i])+delta,0,255));
  }
};
}  // namespace grid_visualizer_pkg
