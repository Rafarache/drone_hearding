#pragma once
#include <cmath>
#include <algorithm>
#include <deque>
namespace grid_visualizer_pkg {
// Measured motion only; DistanceRecovery owns recovery lifetime.
class StallRecovery {
public:
  void reset() { samples_.clear(); }
  bool update(double now,double x,double y,bool eligible,double window,double minimum) {
    if(!eligible || (!samples_.empty() && now<samples_.back().time)) {reset();}
    if(!eligible) {return false;}
    if(!samples_.empty() && now==samples_.back().time) {return false;}
    samples_.push_back({now,x,y});
    while(samples_.size()>1 && samples_[1].time<=now-window) {samples_.pop_front();}
    if(now-samples_.front().time>=window) {
      double extent=0;
      double travel=0,previous_dx=0,previous_dy=0;
      int reversals=0;
      auto anchor=samples_.front();
      for(auto p:samples_) {
        extent=std::max(extent,std::hypot(p.x-x,p.y-y));
        double dx=p.x-anchor.x,dy=p.y-anchor.y,length=std::hypot(dx,dy);
        // Accumulate slow travel without treating sensor jitter as reversals.
        if(length<0.05) {continue;}
        travel+=length;
        if(dx*previous_dx+dy*previous_dy<0) {++reversals;}
        previous_dx=dx;previous_dy=dy;anchor=p;
      }
      const double displacement=std::hypot(x-samples_.front().x,y-samples_.front().y);
      return extent<minimum || (travel>=0.8 && reversals>=2 && displacement<0.25*travel);
    }
    return false;
  }
private:
  struct Sample {double time,x,y;};
  std::deque<Sample> samples_;
};
}
