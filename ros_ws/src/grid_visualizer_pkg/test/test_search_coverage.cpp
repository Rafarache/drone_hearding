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
    ObservationBox box(2,10,2.09);
    assert(std::abs(distance(v.position,v.look_at)-(box.front+box.back)/2)<1e-8);
    assert(a.gain(v.position,std::atan2(v.look_at.y-v.position.y,v.look_at.x-v.position.x),2,10,2.09)>0);
  }
  auto alternative=a.views({0,0},{},free,2,10,2.09,12,
    [&](SearchPoint p) {return distance(p,views[0].position)<0.1;});
  assert(!alternative.empty() && distance(alternative[0].position,views[0].position)>0.1);
  SearchCoverage blank(-10,-10,20,20,0.5);
  assert(!blank.views({0,0},{},free,2,10,2.09).empty());
  assert(a.views({0,0},{},[](SearchPoint) {return true;},2,10,2.09).empty());

  // Leave one fixed patch unfinished. Its viewing side must follow the
  // approach, including looking inward from the outer side of the field.
  SearchCoverage approach(-30,-30,60,60,.5);
  approach.observe({{-30,-30},{30,-30},{30,10},{-30,10}});
  approach.observe({{-30,12},{30,12},{30,30},{-30,30}});
  approach.observe({{-30,10},{10,10},{10,12},{-30,12}});
  approach.observe({{12,10},{30,10},{30,12},{12,12}});
  assert(approach.patch_remaining({11,11},2,10,2.09)==16);
  auto observe_view=[](SearchCoverage & coverage,const SearchView & view,double fov) {
    ObservationBox box(2,10,fov);
    const double yaw=std::atan2(view.look_at.y-view.position.y,view.look_at.x-view.position.x);
    std::vector<SearchPoint> polygon;
    for(auto q:std::vector<SearchPoint>{{box.front,-box.half_width},{box.back,-box.half_width},
        {box.back,box.half_width},{box.front,box.half_width}}) {
      polygon.push_back({view.position.x+q.x*std::cos(yaw)-q.y*std::sin(yaw),
        view.position.y+q.x*std::sin(yaw)+q.y*std::cos(yaw)});
    }
    coverage.observe(polygon);
  };
  for(auto origin:std::vector<SearchPoint>{{-15,11},{24,11},{11,24},{24,24}}) {
    auto planned=approach.views(origin,{},free,2,10,2.09,1);
    assert(planned.size()==1);
    const auto& view=planned.front();
    assert(distance(view.look_at,{11,11})<1e-9);
    const double ax=view.position.x-origin.x,ay=view.position.y-origin.y;
    const double bx=view.look_at.x-origin.x,by=view.look_at.y-origin.y;
    assert(std::abs(ax*by-ay*bx)<1e-8); // Exactly on drone -> patch segment.
    assert(ax*bx+ay*by>0 && distance(origin,view.position)<distance(origin,view.look_at));
    auto observed=approach;
    observe_view(observed,view,2.09);
    assert(observed.patch_remaining(view.look_at,2,10,2.09)==0);
    assert(observed.views(origin,{},free,2,10,2.09,1).empty());
  }
  auto east=approach.views({24,11},{},free,2,10,2.09,1).front();
  auto west=approach.views({-15,11},{},free,2,10,2.09,1).front();
  assert(east.position.x>11 && west.position.x<11);
  auto obstructed=[&](SearchPoint p) {return distance(p,east.position)<.75;};
  for(bool obstacle:{false,true}) {
    auto alternate=approach.views({24,11},{},obstacle ? std::function<bool(SearchPoint)>(obstructed):free,
      2,10,2.09,1,obstacle ? std::function<bool(SearchPoint)>{}:obstructed);
    assert(alternate.size()==1 && distance(alternate.front().position,east.position)>.75);
    assert(alternate.front().position.x>11); // Keep the same approach side if feasible.
    auto observed=approach;
    observe_view(observed,alternate.front(),2.09);
    assert(observed.patch_remaining({11,11},2,10,2.09)==0);
  }
  // Too close to fit the whole patch: a finite step-back viewpoint is needed.
  auto close=approach.views({11.1,11.1},{},free,2,10,2.09,1);
  assert(close.size()==1);
  auto close_observed=approach;
  observe_view(close_observed,close.front(),2.09);
  assert(close_observed.patch_remaining({11,11},2,10,2.09)==0);
  // Narrow footprints may need a different angle to preserve the full patch.
  SearchCoverage narrow(-10,-10,20,20,.5);
  auto narrow_views=narrow.views({0,0},{},free,2,10,.4,12);
  assert(!narrow_views.empty());
  for(const auto& view:narrow_views) {
    auto observed=narrow;
    observe_view(observed,view,.4);
    assert(observed.patch_remaining(view.look_at,2,10,.4)==0);
  }
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

  // A single correctly oriented observation covers the selected fixed patch.
  SearchCoverage fixed(-50,-50,100,100,.5);
  auto first=fixed.views({0,0},{},free,2,10,2.09,1,{},true).front();
  const auto before=fixed.patch_remaining(first.look_at,2,10,2.09);
  const double yaw=std::atan2(first.look_at.y-first.position.y,first.look_at.x-first.position.x);
  assert(fixed.patch_gain(first.look_at,first.position,yaw,2,10,2.09)==before);
  fixed.observe({{first.look_at.x-1,first.look_at.y-1},
    {first.look_at.x,first.look_at.y-1},{first.look_at.x,first.look_at.y+1},
    {first.look_at.x-1,first.look_at.y+1}});
  assert(fixed.patch_remaining(first.look_at,2,10,2.09)>0);
  auto partial=fixed.views({0,0},{},free,2,10,2.09,10000,{},true);
  assert(std::any_of(partial.begin(),partial.end(),[&](auto v) {
    return distance(v.look_at,first.look_at)<1e-8 && distance(v.position,first.position)<1e-8;
  }));
  ObservationBox footprint(2,10,2.09);
  std::vector<SearchPoint> polygon;
  for(auto q:std::vector<SearchPoint>{{footprint.front,-footprint.half_width},
    {footprint.back,-footprint.half_width},{footprint.back,footprint.half_width},
    {footprint.front,footprint.half_width}}) {
    polygon.push_back({first.position.x+q.x*std::cos(yaw)-q.y*std::sin(yaw),
      first.position.y+q.x*std::sin(yaw)+q.y*std::cos(yaw)});
  }
  fixed.observe(polygon);
  assert(fixed.patch_remaining(first.look_at,2,10,2.09)==0);
  SearchCoverage spread(-50,-50,100,100,.5);
  auto north=separation_direction({0,2},{{0,0}},1,2);
  auto south=separation_direction({0,0},{{0,2}},0,2);
  assert(north.x==0 && north.y==1 && south.x==0 && south.y== -1);
  auto overlap0=separation_direction({0,0},{{0,0}},0,2);
  auto overlap1=separation_direction({0,0},{{0,0}},1,2);
  assert(distance(overlap0,overlap1)>1.99);
  assert(distance(separation_direction({0,0},{},0,1),{0,0})==0);
  auto n=spread.views({0,2},{{0,0}},free,2,10,2.09,1,{},true,&north).front();
  auto south_view=spread.views({0,0},{{0,2}},free,2,10,2.09,1,{},true,&south).front();
  assert(n.look_at.y>40 && south_view.look_at.y< -40);

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
