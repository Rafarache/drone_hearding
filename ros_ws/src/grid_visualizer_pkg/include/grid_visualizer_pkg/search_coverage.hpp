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

// Startup heading is opposite the peers' centroid. Coincident drones use
// deterministic equally spaced directions rather than an undefined vector.
inline SearchPoint separation_direction(SearchPoint origin,const std::vector<SearchPoint>& peers,
  int drone_index,int total_drones)
{
  if(peers.empty()) {return {0,0};}
  SearchPoint center{0,0};
  for(auto p:peers) {center.x+=p.x;center.y+=p.y;}
  double dx=origin.x-center.x/peers.size(),dy=origin.y-center.y/peers.size();
  double d=std::hypot(dx,dy);
  if(d<1e-6) {
    double angle=2*std::acos(-1.0)*drone_index/total_drones;
    return {std::cos(angle),std::sin(angle)};
  }
  return {dx/d,dy/d};
}

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

  int patch_step(double near,double far,double fov) const
  {
    ObservationBox box(near,far,fov);
    int step=static_cast<int>(std::floor(std::min({2.0,box.back-box.front-1,
      2*box.half_width-1})/cell_));
    if (step<1) {throw std::invalid_argument("Camera cannot contain a search cell with margin");}
    return step;
  }

  std::size_t patch_remaining(SearchPoint center,double near,double far,double fov) const
  {
    int i=index(center), step=patch_step(near,far,fov);
    if (i<0) {return 0;}
    int r=(i/cols_/step)*step,c=(i%cols_/step)*step;
    std::size_t count=0;
    for(int y=r;y<std::min(rows_,r+step);++y) {
      for(int x=c;x<std::min(cols_,c+step);++x) {count+=!seen_[y*cols_+x];}
    }
    return count;
  }

  std::size_t patch_gain(SearchPoint center,SearchPoint camera,double yaw,
    double near,double far,double fov) const
  {
    int i=index(center),step=patch_step(near,far,fov);
    if(i<0) {return 0;}
    int r=(i/cols_/step)*step,c=(i%cols_/step)*step;
    ObservationBox box(near,far,fov);
    std::size_t count=0;
    for(int y=r;y<std::min(rows_,r+step);++y) {
      for(int x=c;x<std::min(cols_,c+step);++x) {
        if(seen_[y*cols_+x]) {continue;}
        auto q=point(y*cols_+x);
        double dx=q.x-camera.x,dy=q.y-camera.y;
        double f=dx*std::cos(yaw)+dy*std::sin(yaw);
        double l=-dx*std::sin(yaw)+dy*std::cos(yaw);
        if(f>=box.front-1e-9 && f<=box.back+1e-9 && std::abs(l)<=box.half_width+1e-9) {++count;}
      }
    }
    return count;
  }

  std::vector<SearchView> views(
    SearchPoint drone, const std::vector<SearchPoint>& peers,
    const std::function<bool(SearchPoint)>& blocked,
    double near, double far, double fov, int max_views=12,
    const std::function<bool(SearchPoint)>& retired={}, bool border_first=false, const SearchPoint* preferred_direction=nullptr) const
  {
    if (count_==seen_.size() || max_views<=0) {return {};}
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
    // Cache the fixed patch bounds, not a border-facing camera position.
    // A viewpoint is chosen from this drone's approach side at selection time;
    // the node then retains it until the patch is covered or invalidated.
    if (patches_.empty() || camera_near_!=near || camera_far_!=far || camera_fov_!=fov) {
      std::vector<Patch> patches;
      const int step=patch_step(near,far,fov);
      for(int r=0;r<rows_;r+=step) {for(int c=0;c<cols_;c+=step) {
        const double left=min_x_+c*cell_,right=std::min(max_x_,min_x_+(c+step)*cell_);
        const double bottom=min_y_+r*cell_,top=std::min(max_y_,min_y_+(r+step)*cell_);
        patches.push_back({{(left+right)/2,(bottom+top)/2},(right-left)/2,(top-bottom)/2});
      }}
      patches_=std::move(patches);camera_near_=near;camera_far_=far;camera_fov_=fov;
    }
    const ObservationBox box(near,far,fov);
    const double offset=(box.front+box.back)/2;
    std::vector<SearchView> result;
    int best_band=std::numeric_limits<int>::max();
    for(const auto& patch:patches_) {
      const auto target=patch.center;
      if(!patch_remaining(target,near,far,fov)) {continue;}
      const int band=border_first ? static_cast<int>(border_depth(target)/(far/2)):0;
      if(band>best_band) {continue;}
      const double bearing=std::atan2(target.y-drone.y,target.x-drone.x);
      const double pi=std::acos(-1.0);
      SearchPoint view{};
      int j=-1;
      auto choose_view=[&](double heading) {
        const double cosine=std::cos(heading),sine=std::sin(heading);
        SearchPoint candidate{target.x-offset*cosine,target.y-offset*sine};
        const int candidate_index=index(candidate);
        if(candidate_index<0 || blocked(candidate) || (retired && retired(candidate)) ||
          !std::isfinite(costs[candidate_index])) {return false;}
        // Verify all four physical patch corners with the same 0.5 m margin
        // used to size the patches. Rotation must not leave an unseen sliver.
        for(double dx:{-patch.half_width,patch.half_width}) {
          for(double dy:{-patch.half_height,patch.half_height}) {
            const double forward=offset+dx*cosine+dy*sine;
            const double lateral=-dx*sine+dy*cosine;
            if(forward<box.front+0.5-1e-9 || forward>box.back-0.5+1e-9 ||
              std::abs(lateral)>box.half_width-0.5+1e-9) {return false;}
          }
        }
        view=candidate;j=candidate_index;return true;
      };
      // The common case needs one candidate, without allocating/sorting angles.
      // Only a blocked or invalid approach ray needs angular alternatives.
      if(!choose_view(bearing)) {
        std::vector<double> headings;
        for(int k=1;k<8;++k) {
          headings.push_back(bearing+k*pi/8);headings.push_back(bearing-k*pi/8);
        }
        headings.push_back(bearing+pi);
        // Cardinal candidates preserve views for very narrow footprints.
        for(int k=0;k<4;++k) {headings.push_back(k*pi/2);}
        std::stable_sort(headings.begin(),headings.end(),[&](double a,double b) {
          return std::abs(angle_difference(a,bearing))<std::abs(angle_difference(b,bearing));
        });
        for(double heading:headings) {if(choose_view(heading)) {break;}}
      }
      if(j<0) {continue;}
      if(band<best_band) {result.clear();best_band=band;}
      double crowd=1;
      for(auto peer:peers) {crowd+=std::max(0.0,1-distance(view,peer)/far);}
      double preference=1;
      if(preferred_direction) {
        double dx=target.x-drone.x,dy=target.y-drone.y,d=std::hypot(dx,dy);
        if(d>0) {preference=std::exp(4*(dx*preferred_direction->x+dy*preferred_direction->y)/d);}
      }
      result.push_back({view,target,preference/((1+costs[j]/far)*crowd),
        border_first ? (band+1)*far/2:inf});
    }
    std::stable_sort(result.begin(),result.end(),[](const SearchView& a,const SearchView& b) {
      return a.score>b.score;
    });
    if(static_cast<int>(result.size())>max_views) {result.resize(max_views);}
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
  struct Patch {SearchPoint center;double half_width,half_height;};
  mutable std::vector<Patch> patches_;
  mutable double camera_near_{-1},camera_far_{-1},camera_fov_{-1};
};

inline double absorbing_value(double neighbor_sum,double absorption)
{return neighbor_sum/(4.0+absorption);}

// Over-relaxation may overshoot even when its inputs are normalized. Keep every
// intermediate iterate in the potential range; convergence still checks the
// original absorption equation's residual.
inline double relaxed_absorbing_value(double old_value,double equilibrium_value,double factor)
{return std::clamp(old_value+factor*(equilibrium_value-old_value),0.0,1.0);}

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
