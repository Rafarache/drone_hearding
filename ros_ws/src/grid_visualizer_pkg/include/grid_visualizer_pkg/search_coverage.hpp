#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace grid_visualizer_pkg
{
struct SearchPoint {double x; double y;};
inline double distance(SearchPoint a, SearchPoint b) {return std::hypot(a.x-b.x, a.y-b.y);}
inline double angle_difference(double a, double b) {return std::atan2(std::sin(a-b), std::cos(a-b));}

struct SearchView
{
  SearchPoint position;
  SearchPoint look_at;
  double score;
  double max_border_depth{std::numeric_limits<double>::infinity()};
};

// Same conservative camera box as yolo_pkg.search_observation. Coordinates are
// forward/lateral in the camera frame; all corners lie inside the annular FOV.
struct ObservationBox
{
  double front, back, half_width;
  ObservationBox(double near, double far, double fov)
  {
    front=std::max(near,std::min(4.0,far*0.4));
    back=std::max(5.0,(front+std::sqrt(front*front+8*far*far))/4);
    half_width=std::sqrt(far*far-back*back);
    if (fov<std::acos(-1.0)) {half_width=std::min(half_width,front*std::tan(fov/2));}
  }
};

// Independent, permanent observation memory. No ROS or navigation-grid ownership.
class SearchCoverage
{
public:
  SearchCoverage(double min_x, double min_y, double width, double height, double cell)
  : min_x_(min_x), min_y_(min_y), max_x_(min_x+width), max_y_(min_y+height), cell_(cell),
    cols_(static_cast<int>(std::ceil(width/cell))),
    rows_(static_cast<int>(std::ceil(height/cell))), seen_(cols_*rows_, 0) {}

  SearchPoint point(int index) const
  {return {min_x_+(index%cols_+0.5)*cell_, min_y_+(index/cols_+0.5)*cell_};}
  int index(SearchPoint p) const
  {
    int c=static_cast<int>(std::floor((p.x-min_x_)/cell_));
    int r=static_cast<int>(std::floor((p.y-min_y_)/cell_));
    return c>=0 && c<cols_ && r>=0 && r<rows_ ? r*cols_+c : -1;
  }
  bool seen(SearchPoint p) const {int i=index(p); return i>=0 && seen_[i];}
  int rows() const {return rows_;}
  int cols() const {return cols_;}
  bool seen_cell(int row, int col) const
  {return row>=0 && row<rows_ && col>=0 && col<cols_ && seen_[row*cols_+col];}
  std::size_t count() const {return count_;}
  double fraction() const {return static_cast<double>(count_)/seen_.size();}
  double border_depth(SearchPoint p) const
  {return std::min({p.x-min_x_, max_x_-p.x, p.y-min_y_, max_y_-p.y});}

  static bool inside(SearchPoint p, const std::vector<SearchPoint>& polygon)
  {
    bool result=false;
    for (std::size_t i=0, j=polygon.size()-1; i<polygon.size(); j=i++) {
      const auto a=polygon[i], b=polygon[j];
      const double cross=(p.x-a.x)*(b.y-a.y)-(p.y-a.y)*(b.x-a.x);
      if (std::abs(cross)<1e-8 && p.x>=std::min(a.x,b.x)-1e-8 &&
        p.x<=std::max(a.x,b.x)+1e-8 && p.y>=std::min(a.y,b.y)-1e-8 &&
        p.y<=std::max(a.y,b.y)+1e-8) {return true;}
      if ((a.y>p.y)!=(b.y>p.y) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x) {
        result=!result;
      }
    }
    return result;
  }

  std::size_t observe(const std::vector<SearchPoint>& polygon)
  {
    if (polygon.size()<3) {return 0;}
    double x0=polygon[0].x, x1=x0, y0=polygon[0].y, y1=y0;
    for (auto p:polygon) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return 0;}
      x0=std::min(x0,p.x); x1=std::max(x1,p.x);
      y0=std::min(y0,p.y); y1=std::max(y1,p.y);
    }
    const auto bounds=window(x0,y0,x1,y1);
    std::size_t added=0;
    for (int r=bounds[1]; r<=bounds[3]; ++r) {
      for (int c=bounds[0]; c<=bounds[2]; ++c) {
        const int i=r*cols_+c;
        if (!seen_[i] && inside(point(i),polygon)) {seen_[i]=1; ++added;}
      }
    }
    count_+=added;
    return added;
  }

  std::size_t gain(SearchPoint p, double yaw, double near, double far, double fov,
    double max_border_depth=std::numeric_limits<double>::infinity()) const
  {
    const auto bounds=window(p.x-far,p.y-far,p.x+far,p.y+far);
    const ObservationBox box(near,far,fov);
    const double cosine=std::cos(yaw), sine=std::sin(yaw);
    std::size_t result=0;
    for (int r=bounds[1]; r<=bounds[3]; ++r) {
      for (int c=bounds[0]; c<=bounds[2]; ++c) {
        int i=r*cols_+c;
        if (seen_[i]) {continue;}
        auto q=point(i);
        if (border_depth(q)>=max_border_depth) {continue;}
        const double dx=q.x-p.x, dy=q.y-p.y;
        const double forward=dx*cosine+dy*sine, lateral=-dx*sine+dy*cosine;
        if (forward>=box.front-1e-9 && forward<=box.back+1e-9 &&
          std::abs(lateral)<=box.half_width+1e-9) {++result;}
      }
    }
    return result;
  }

  std::vector<SearchView> views(
    SearchPoint drone, const std::vector<SearchPoint>& peers,
    const std::function<bool(SearchPoint)>& blocked,
    double near, double far, double fov, int max_views=12,
    const std::function<bool(SearchPoint)>& retired={}, bool border_first=false) const
  {
    if (count_==seen_.size() || max_views<=0 || (!border_first && count_==0)) {return {};}
    // One global wavefront per planning update; disallow diagonal corner cutting.
    std::vector<uint8_t> free(seen_.size());
    for (int i=0; i<static_cast<int>(free.size()); ++i) {free[i]=!blocked(point(i));}
    int start=index(drone);
    if (start<0) {return {};}
    free[start]=1;
    const double inf=std::numeric_limits<double>::infinity();
    std::vector<double> costs(seen_.size(),inf);
    using Entry=std::pair<double,int>;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
    costs[start]=0; queue.emplace(0,start);
    while (!queue.empty()) {
      auto [cost,i]=queue.top(); queue.pop();
      if (cost>costs[i]) {continue;}
      neighbors(i,[&](int j,int dr,int dc) {
        if (!free[j] || (dr && dc && (!free[i+dr*cols_] || !free[i+dc]))) {return;}
        double next=cost+cell_*(dr && dc ? 1.4142135623730951:1.0);
        if (next<costs[j]) {costs[j]=next; queue.emplace(next,j);}
      });
    }
    struct Patch {int band; std::vector<int> cells;};
    std::vector<Patch> patches;
    // Camera-range tiles break a whole border ring into useful local targets.
    // Bands are half a camera range deep and are reconsidered every cycle.
    const double band_width=far/2;
    if (border_first) {
      std::map<std::tuple<int,int,int>,std::vector<int>> tiles;
      for (int i=0; i<static_cast<int>(seen_.size()); ++i) {
        auto p=point(i);
        double depth=border_depth(p);
        if (seen_[i] || depth<0) {continue;}
        int band=static_cast<int>(std::floor(depth/band_width));
        int tile_row=static_cast<int>(std::floor((p.y-min_y_)/far));
        int tile_col=static_cast<int>(std::floor((p.x-min_x_)/far));
        tiles[{band,tile_row,tile_col}].push_back(i);
      }
      for (auto& entry:tiles) {
        patches.push_back({std::get<0>(entry.first),std::move(entry.second)});
      }
    } else {
      std::vector<uint8_t> frontier(seen_.size(),0), visited(seen_.size(),0);
      for (int i=0; i<static_cast<int>(seen_.size()); ++i) {
        if (seen_[i]) {continue;}
        neighbors(i,[&](int j,int,int) {if (seen_[j]) {frontier[i]=1;}});
      }
      for (int i=0; i<static_cast<int>(frontier.size()); ++i) {
        if (!frontier[i] || visited[i]) {continue;}
        std::vector<int> cluster{i}; visited[i]=1;
        for (std::size_t k=0; k<cluster.size(); ++k) {
          neighbors(cluster[k],[&](int j,int,int) {
            if (frontier[j] && !visited[j]) {visited[j]=1; cluster.push_back(j);}
          });
        }
        patches.push_back({-1,std::move(cluster)});
      }
    }
    std::vector<SearchView> result;
    int selected_band=-1;
    for (const auto& patch:patches) {
      // Try a deeper band only if no reachable positive-gain outer view exists.
      if (border_first && selected_band>=0 && patch.band!=selected_band) {break;}
      const auto& cluster=patch.cells;
      const double max_depth=border_first ? (patch.band+1)*band_width:inf;
      SearchPoint center{0,0};
      for (int current:cluster) {
        auto p=point(current); center.x+=p.x; center.y+=p.y;
      }
      center.x/=cluster.size(); center.y/=cluster.size();
      int representative=*std::min_element(cluster.begin(),cluster.end(),[&](int a,int b) {
        return distance(point(a),center)<distance(point(b),center);
      });
      const auto target=point(representative);
      SearchView best{{0,0},target,0,max_depth};
      for (int k=0; k<8; ++k) {
        double angle=k*std::acos(-1.0)/4;
        SearchPoint view{target.x+5*std::cos(angle),target.y+5*std::sin(angle)};
        int j=index(view);
        if (j<0 || !free[j] || blocked(view) || (retired && retired(view)) || !std::isfinite(costs[j])) {continue;}
        double yaw=std::atan2(target.y-view.y,target.x-view.x);
        double novelty=gain(view,yaw,near,far,fov,max_depth);
        double crowd=1;
        for (auto peer:peers) {crowd+=std::max(0.0,1-distance(view,peer)/far);}
        double score=novelty/(1+costs[j]/far)/crowd;
        if (score>best.score) {best={view,target,score,max_depth};}
      }
      if (best.score>0) {result.push_back(best); selected_band=patch.band;}
    }
    std::sort(result.begin(),result.end(),[](const SearchView& a,const SearchView& b) {
      if (a.score!=b.score) {return a.score>b.score;}
      return std::tie(a.position.x,a.position.y)<std::tie(b.position.x,b.position.y);
    });
    if (static_cast<int>(result.size())>max_views) {result.resize(max_views);}
    return result;
  }

private:
  std::vector<int> window(double x0,double y0,double x1,double y1) const
  {
    // Clamp doubles first, avoiding integer overflow for malformed/extreme polygons.
    auto c=[&](double x) {return static_cast<int>(std::floor(std::clamp((x-min_x_)/cell_,0.0,double(cols_-1))));};
    auto r=[&](double y) {return static_cast<int>(std::floor(std::clamp((y-min_y_)/cell_,0.0,double(rows_-1))));};
    return {c(x0),r(y0),c(x1),r(y1)};
  }
  template<typename F> void neighbors(int i,F f) const
  {
    int r=i/cols_,c=i%cols_;
    for (int dr=-1; dr<=1; ++dr) {for (int dc=-1; dc<=1; ++dc) {
      if ((dr || dc) && r+dr>=0 && r+dr<rows_ && c+dc>=0 && c+dc<cols_) {
        f(i+dr*cols_+dc,dr,dc);
      }
    }}
  }
  double min_x_,min_y_,max_x_,max_y_,cell_;
  int cols_,rows_;
  std::vector<uint8_t> seen_;
  std::size_t count_{0};
};

inline double absorbing_value(double neighbor_sum,double absorption)
{return neighbor_sum/(4.0+absorption);}

// Geometric track association preserves the existing PoseArray interface.
class SearchBalance
{
public:
  std::pair<double,double> update(double now,std::size_t coverage,
    const std::vector<SearchPoint>& cows,SearchPoint goal,bool searching,bool herding)
  {
    if (!initialized_ || now<last_time_) {
      initialized_=true; search_progress_=herd_progress_=now; tracks_.clear();
    }
    last_time_=now;
    if (coverage>last_coverage_) {search_progress_=now;}
    last_coverage_=coverage;
    std::vector<std::tuple<double,std::size_t,std::size_t>> pairs;
    for (std::size_t i=0;i<tracks_.size();++i) {for (std::size_t j=0;j<cows.size();++j) {
      double d=distance(tracks_[i].position,cows[j]);
      if (d<=3) {pairs.emplace_back(d,i,j);}
    }}
    std::sort(pairs.begin(),pairs.end());
    std::vector<int> matches(cows.size(),-1);
    std::vector<bool> used(tracks_.size(),false);
    for (auto [d,i,j]:pairs) {
      (void)d;
      if (!used[i] && matches[j]<0) {used[i]=true; matches[j]=i;}
    }
    std::vector<Track> next;
    for (std::size_t j=0;j<cows.size();++j) {
      Track t=matches[j]>=0 ? tracks_[matches[j]] : Track{};
      t.position=cows[j];
      t.history.emplace_back(now,distance(cows[j],goal));
      while (!t.history.empty() && t.history.front().first<now-10) {t.history.pop_front();}
      if (!t.history.empty() && now-t.history.front().first>=9.5) {
        std::vector<double> old,recent;
        for (auto sample:t.history) {(sample.first<now-5 ? old:recent).push_back(sample.second);}
        if (!old.empty() && !recent.empty() && median(old)-median(recent)>=0.2) {herd_progress_=now;}
      }
      next.push_back(std::move(t));
    }
    tracks_=std::move(next);
    if (!searching) {search_progress_=now;}
    if (!herding) {herd_progress_=now;}
    double a=searching ? 1+std::clamp((now-search_progress_)/30.0,0.0,1.0):0;
    double b=herding ? 1+std::clamp((now-herd_progress_)/30.0,0.0,1.0):0;
    return a+b>0 ? std::make_pair(a/(a+b),b/(a+b)):std::make_pair(0.0,0.0);
  }
private:
  struct Track {SearchPoint position{0,0}; std::deque<std::pair<double,double>> history;};
  static double median(std::vector<double>& v) {std::sort(v.begin(),v.end());return v[v.size()/2];}
  std::vector<Track> tracks_;
  bool initialized_{false};
  double last_time_{0},search_progress_{0},herd_progress_{0};
  std::size_t last_coverage_{0};
};
}  // namespace grid_visualizer_pkg
