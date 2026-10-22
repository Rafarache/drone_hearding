#include "grid_visualizer_pkg/search_coverage.hpp"
#include <cassert>
#include <iostream>

using namespace grid_visualizer_pkg;

int main()
{
  // The inexpensive gain calculation must agree with rasterizing the actual
  // four-corner wire footprint, including rotation, map clipping and old cells.
  for (double yaw : {0.0, 0.37, 1.5707963267948966, -2.3}) {
    for (double fov : {0.1, 2.09, 6.283185307179586}) {
      SearchCoverage expected(-10,-10,20,20,.5), actual(-10,-10,20,20,.5);
      expected.observe({{0,0},{4,0},{4,4},{0,4}});
      actual.observe({{0,0},{4,0},{4,4},{0,4}});
      ObservationBox box(2,10,fov);
      std::vector<SearchPoint> corners;
      for (auto p : std::vector<SearchPoint>{{box.front,-box.half_width},
        {box.back,-box.half_width},{box.back,box.half_width},{box.front,box.half_width}}) {
        corners.push_back({3+p.x*std::cos(yaw)-p.y*std::sin(yaw),
          4+p.x*std::sin(yaw)+p.y*std::cos(yaw)});
      }
      assert(expected.gain({3,4},yaw,2,10,fov)==actual.observe(corners));
      assert(actual.observe(corners)==0);
    }
  }
  SearchCoverage a(-10,-10,20,20,0.5), b(-10,-10,20,20,0.5);
  std::vector<SearchPoint> patch{{-2,-2},{2,-2},{2,2},{-2,2}};
  assert(a.observe(patch)==64);
  assert(a.observe(patch)==0);
  b.observe(patch);
  assert(a.count()==b.count());
  assert(a.seen({0,0}) && !a.seen({3,3}));
  assert(a.observe({{NAN,0},{0,0},{1,1}})==0);
  assert(a.observe({{100,100},{101,100},{101,101}})==0);
  auto gain=a.gain({0,0},0,2,8,2.09);
  assert(gain>0 && gain==a.gain({0,0},2*std::acos(-1.0),2,8,2.09));
  auto free=[](SearchPoint) {return false;};
  auto views=a.views({0,0},{},free,2,10,2.09);
  assert(!views.empty());
  for (auto v:views) {
    assert(std::abs(distance(v.position,v.look_at)-5)<1e-8);
    assert(a.gain(v.position,std::atan2(v.look_at.y-v.position.y,v.look_at.x-v.position.x),2,10,2.09)>0);
  }
  auto alternative=a.views({0,0},{},free,2,10,2.09,12,
    [&](SearchPoint p) {return distance(p,views[0].position)<0.1;});
  assert(!alternative.empty() && distance(alternative[0].position,views[0].position)>0.1);
  SearchCoverage blank(-10,-10,20,20,0.5);
  assert(blank.views({0,0},{},free,2,10,2.09).empty());
  assert(a.views({0,0},{},[](SearchPoint) {return true;},2,10,2.09).empty());
  // Boundary/corner observations can be made from inward viewpoints.
  SearchCoverage edge(0,0,20,20,0.5);
  edge.observe({{0,0},{4,0},{4,4},{0,4}});
  assert(!edge.views({2,2},{},free,2,10,2.09).empty());

  // A central starting footprint must not pull outside-in search toward the
  // nearby frontier: all targets belong to the outermost unvisited 5 m band.
  SearchCoverage outside_in(-50,-30,100,60,0.5);
  outside_in.observe(patch);
  auto border_views=outside_in.views({0,0},{},free,2,10,2.09,12,{},true);
  assert(!border_views.empty());
  for (const auto& view:border_views) {
    assert(outside_in.border_depth(view.look_at)<5);
    assert(view.max_border_depth==5);
    assert(distance({0,0},view.position)>19);
    assert(outside_in.gain(view.position,
      std::atan2(view.look_at.y-view.position.y,view.look_at.x-view.position.x),
      2,10,2.09,view.max_border_depth)>0);
  }
  assert(outside_in.gain({0,0},0,2,10,2.09,5)==0);
  // If all outer viewpoints are temporarily retired, a deeper reachable band
  // remains useful instead of stopping the entire search.
  auto deeper=outside_in.views({0,0},{},free,2,10,2.09,12,
    [&](SearchPoint p) {return outside_in.border_depth(p)<10;},true);
  assert(!deeper.empty() && deeper.front().max_border_depth>5);
  assert(outside_in.views({0,0},{},free,2,10,2.09,12,
    [](SearchPoint) {return true;},true).empty());
  outside_in.observe({{-50,-30},{50,-30},{50,-25},{-50,-25}});
  outside_in.observe({{-50,25},{50,25},{50,30},{-50,30}});
  outside_in.observe({{-50,-25},{-45,-25},{-45,25},{-50,25}});
  outside_in.observe({{45,-25},{50,-25},{50,25},{45,25}});
  border_views=outside_in.views({0,0},{},free,2,10,2.09,12,{},true);
  assert(!border_views.empty());
  for (const auto& view:border_views) {
    assert(view.max_border_depth==10);
    assert(outside_in.border_depth(view.look_at)>=5 && outside_in.border_depth(view.look_at)<10);
  }
  outside_in.observe({{-50,-30},{50,-30},{50,30},{-50,30}});
  assert(outside_in.views({0,0},{},free,2,10,2.09,12,{},true).empty());

  // Solve a small domain and verify the absorption maximum principle and
  // monotonic response to stronger targets, rather than just its update formula.
  auto solve=[](double strength) {
    const int n=15;
    std::vector<double> u(n*n,1);
    for (int k=0;k<1500;++k) {
      for (int r=1;r<n-1;++r) {for (int c=1;c<n-1;++c) {
        double sum=u[(r-1)*n+c]+u[(r+1)*n+c]+u[r*n+c-1]+u[r*n+c+1];
        u[r*n+c]=absorbing_value(sum, r==7 && c==7 ? strength:0);
      }}
    }
    return u;
  };
  auto weak=solve(1),strong=solve(8);
  for (std::size_t i=0;i<weak.size();++i) {
    assert(strong[i]>0 && strong[i]<=1 && strong[i]<=weak[i]+1e-10);
  }
  assert(strong[7*15+7]<strong[7*15+8]);
  SearchBalance balance;
  auto weights=balance.update(0,0,{{10,0}},{0,0},true,true);
  assert(weights.first==0.5 && weights.second==0.5);
  weights=balance.update(30,1,{{10,0}},{0,0},true,true);
  assert(std::abs(weights.first-1.0/3)<1e-8 && weights.second>0);
  weights=balance.update(31,1,{}, {0,0},true,false);
  assert(weights.first==1 && weights.second==0);
  weights=balance.update(0,0,{}, {0,0},true,true);
  assert(weights.first==0.5);
  SearchBalance progress;
  for (int i=0;i<=30;++i) {
    weights=progress.update(i,i,{{20-0.3*i,0}},{0,0},true,true);
  }
  assert(std::abs(weights.first-0.5)<1e-8);
  std::cout << "coverage, viewpoints, absorption and progress checks passed\n";
}
