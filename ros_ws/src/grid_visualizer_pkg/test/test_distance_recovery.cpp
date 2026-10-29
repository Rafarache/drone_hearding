#include "grid_visualizer_pkg/distance_recovery.hpp"
#include "grid_visualizer_pkg/stall_recovery.hpp"
#include <cassert>
#include <iostream>

using namespace grid_visualizer_pkg;

RecoveryGrid frame(RecoveryPoint center={0,0})
{
  return {40,40,.5,center,std::vector<std::uint8_t>(1600,0)};
}
RecoveryTarget target(const RecoveryGrid& grid,RecoveryPoint point,bool search=false,int patch=-1,double strength=.4)
{
  return {point,point,search,patch,grid.index(point),strength,10*(1-strength)};
}
bool event(const RecoveryResult& result,RecoveryEvent wanted)
{
  return std::find(result.events.begin(),result.events.end(),wanted)!=result.events.end();
}

void test_target_retention_and_safety()
{
  auto grid=frame();
  RecoverySettings settings;
  DistanceRecovery recovery;
  auto cow=target(grid,{0,8}),search=target(grid,{0,-8},true,7,.6);
  auto result=recovery.update(0,{0,0},grid,{cow,search},{0,0},true,true,settings);
  assert(result.active && result.has_step && recovery.target()->search);
  assert(result.movement.y<0 && std::hypot(result.movement.x,result.movement.y)<=.5+1e-9);
  // A cheaper competing goal must not steal active recovery.
  cow.source_cost=0;
  result=recovery.update(1,{0,-1.2},grid,{cow,search},{0,1},true,false,settings);
  assert(result.active && recovery.target()->search);
  assert(result.movement.y<0); // More than 1 m travelled does not release it.
  // Recenter the grid: retain the world target, not the old source cell.
  grid=frame({0,-3});cow=target(grid,{0,5});search=target(grid,{0,-8},true,7,.6);
  result=recovery.update(2,{0,-3},grid,{cow,search},{0,1},true,false,settings);
  assert(result.active && recovery.target()->patch_id==7 && result.movement.y<0);
  result=recovery.update(3,{0,-3},grid,{cow},{0,1},true,false,settings);
  assert(event(result,RecoveryEvent::invalidated) && recovery.target() && !recovery.target()->search);
  // No reachable candidates means an explicit recovery hold.
  cow.strength=0;
  result=recovery.update(4,{0,-3},grid,{cow},{0,1},true,false,settings);
  assert(result.active && !result.has_step && !recovery.target());
  result=recovery.update(1,{0,-3},grid,{cow},{0,1},true,false,settings);
  assert(event(result,RecoveryEvent::clock_reset) && recovery.phase()==RecoveryPhase::idle);

  // A diagonal cannot cross even one blocked side cell.
  grid=frame();
  const int start=grid.index({0,0});
  grid.blocked[start-40]=1;grid.blocked[start+1]=1;
  auto costs=recovery_distances(grid,{{start,0}});
  assert(costs[start-40+1]>.5*std::sqrt(2.0));
  assert(!grid.edge(start/40,start%40,-1,1));
  assert(!recovery_segment_free(grid,grid.position(start),grid.position(start-40+1)));
  // Enclose the source completely: unreachable candidates cannot win.
  for(int dr=-1;dr<=1;++dr) {for(int dc=-1;dc<=1;++dc) {
    if(dr || dc) {grid.blocked[start+dr*40+dc]=1;}
  }}
  recovery.reset();
  result=recovery.update(0,{0,0},grid,{target(grid,{0,5})},{0,1},false,true,settings);
  assert(result.active && !result.has_step && !recovery.target());
}

void test_obstacle_detour_and_watch_expiry()
{
  auto grid=frame();
  for(int col=1;col<39;++col) {if(col!=30) {grid.blocked[15*40+col]=1;}}
  const auto goal=target(grid,{0,5},true,10,.6);
  RecoverySettings settings;
  DistanceRecovery recovery;
  RecoveryPoint position{0,0};
  bool arrived=false;
  for(int tick=0;tick<150;++tick) {
    auto result=recovery.update(tick*.5,position,grid,{goal},{0,0},true,tick==0,settings);
    if(event(result,RecoveryEvent::arrived)) {arrived=true;break;}
    assert(result.active && result.has_step);
    RecoveryPoint next{position.x+result.movement.x,position.y+result.movement.y};
    assert(recovery_segment_free(grid,position,next));
    position=next;
  }
  assert(arrived);
  grid=frame();recovery.reset();
  const auto east=target(grid,{8,0});
  for(int tick=0;tick<=4;++tick) {
    recovery.update(tick,{tick*.4,0},grid,{east},{1,0},true,tick==0,settings);
  }
  assert(recovery.phase()==RecoveryPhase::watching);
  auto result=recovery.update(10,{3,0},grid,{east},{1,0},true,false,settings);
  assert(event(result,RecoveryEvent::watch_ended) && recovery.phase()==RecoveryPhase::idle);
  // A moving cow may reset distance references, but cannot extend the watch.
  for(int tick=11;tick<=15;++tick) {
    recovery.update(tick,{(tick-11)*.4,0},grid,{east},{1,0},true,tick==11,settings);
  }
  assert(recovery.phase()==RecoveryPhase::watching);
  auto moving=target(grid,{8.6,0});
  recovery.update(16,{2,0},grid,{moving},{1,0},true,false,settings);
  result=recovery.update(21,{3,0},grid,{moving},{1,0},true,false,settings);
  assert(event(result,RecoveryEvent::watch_ended));
}

void test_handoff_and_watch()
{
  auto grid=frame();
  const auto goal=target(grid,{8,0});
  RecoverySettings settings;
  DistanceRecovery recovery;
  auto result=recovery.update(0,{0,0},grid,{goal},{1,0},true,true,settings);
  assert(result.active);
  // Direction agreement alone, with no measured travel, cannot release.
  for(int t=1;t<=6;++t) {
    result=recovery.update(t,{0,0},grid,{goal},{1,0},true,false,settings);
    assert(result.active);
  }
  for(int t=7;t<=9;++t) {
    result=recovery.update(t,{(t-6)*.4,0},grid,{goal},{1,0},true,false,settings);
  }
  assert(!result.active && event(result,RecoveryEvent::handed_off));
  assert(recovery.phase()==RecoveryPhase::watching);
  recovery.update(10.5,{1.6,0},grid,{goal},{-1,0},true,false,settings);
  result=recovery.update(11.5,{1.4,0},grid,{goal},{-1,0},true,false,settings);
  assert(result.active && event(result,RecoveryEvent::reactivated));
  // Qualification is interrupted by a bad field, restarting the 3 s window.
  for(int t=12;t<=14;++t) {
    result=recovery.update(t,{1.4+(t-11.5)*.4,0},grid,{goal},{1,0},true,false,settings);
    assert(result.active);
  }
  recovery.update(14.5,{2.6,0},grid,{goal},{1,0},false,false,settings);
  for(int t=15;t<=17;++t) {
    result=recovery.update(t,{2.6+(t-14.5)*.4,0},grid,{goal},{1,0},true,false,settings);
    assert(result.active);
  }
  result=recovery.update(18,{4,0},grid,{goal},{1,0},true,false,settings);
  assert(event(result,RecoveryEvent::handed_off));
  result=recovery.update(18.5,{2.9,0},grid,{goal},{1,0},true,false,settings);
  assert(result.active && event(result,RecoveryEvent::reactivated));
  // Target arrival clears memory independently of the handoff timer.
  result=recovery.update(19,{7.6,0},grid,{goal},{1,0},true,false,settings);
  assert(event(result,RecoveryEvent::arrived) && recovery.phase()==RecoveryPhase::idle);
}

void test_moving_target_and_off_window()
{
  auto grid=frame();
  RecoverySettings settings;
  DistanceRecovery recovery;
  auto goal=target(grid,{8,0});
  recovery.update(0,{0,0},grid,{goal},{1,0},true,true,settings);
  recovery.update(1,{.4,0},grid,{goal},{1,0},true,false,settings);
  recovery.update(2,{.8,0},grid,{goal},{1,0},true,false,settings);
  goal=target(grid,{8.6,0});
  auto result=recovery.update(3,{1.2,0},grid,{goal},{1,0},true,false,settings);
  assert(result.active && !event(result,RecoveryEvent::handed_off));
  assert(recovery.target()->position.x==8.6);
  result=recovery.update(4,{1.6,0},grid,{goal},{1,0},true,false,settings);
  assert(result.active); // Moving the goal restarted the agreement window.
  recovery.reset();
  goal=target(grid,{30,0},true,12,.6);
  goal.cell=grid.index({9,0});
  result=recovery.update(0,{0,0},grid,{goal},{0,0},true,true,settings);
  assert(result.active && result.has_step && recovery.target()->position.x==30);
  // Reaching the proxy's old global position is not reaching the true target.
  grid=frame({9,0});goal.cell=grid.index({18,0});
  result=recovery.update(30,{9,0},grid,{goal},{-1,0},true,false,settings);
  assert(result.active && !event(result,RecoveryEvent::arrived));
}

void test_closed_loop_saddle()
{
  RecoverySettings settings;
  DistanceRecovery recovery;
  StallRecovery stalls;
  RecoveryPoint position{0,0},velocity{0,0},waypoint{0,0};
  bool entered=false,handed_off=false;
  double max_escape=0;
  // 10 Hz measured dynamics, 2 Hz planner, 0.4 m/s and 0.5 m/s^2 limits,
  // matching the position controller's tolerance and slowdown calculation.
  for(int tick=0;tick<800;++tick) {
    double now=tick*.1;
    if(tick%5==0) {
      auto grid=frame(position);
      auto north=target(grid,{0,8}),south=target(grid,{0,-8},true,8,.6);
      // A controller-limit-cycle fixture: opposing relaxed commands near the
      // saddle; once escaped, the normal field consistently points south.
      RecoveryPoint normal=max_escape<2 ? RecoveryPoint{0,(tick/10)%2 ? 1.0:-1.0} : RecoveryPoint{0,-1};
      bool stalled=stalls.update(now,position.x,position.y,
        recovery.phase()!=RecoveryPhase::recovering,6,.4);
      auto result=recovery.update(now,position,grid,{north,south},normal,true,stalled,settings);
      entered=entered || event(result,RecoveryEvent::entered);
      handed_off=handed_off || event(result,RecoveryEvent::handed_off);
      if(recovery.target()) {assert(recovery.target()->search);}
      if(result.active) {
        assert(result.has_step);
        assert(std::hypot(result.movement.x,result.movement.y)<=.5+1e-9);
        waypoint={position.x+result.movement.x,position.y+result.movement.y};
      } else {waypoint={position.x+normal.x*.5,position.y+normal.y*.5};}
      if(recovery_distance(position,south.position)<=.5 && entered) {break;}
    }
    double dx=waypoint.x-position.x,dy=waypoint.y-position.y,distance=std::hypot(dx,dy);
    double speed=.4*std::min(1.0,std::max(0.0,distance-.1)/.4);
    RecoveryPoint desired{distance>0 ? speed*dx/distance:0,distance>0 ? speed*dy/distance:0};
    double change=recovery_distance(desired,velocity),scale=change>.05 ? .05/change:1;
    velocity.x+=(desired.x-velocity.x)*scale;velocity.y+=(desired.y-velocity.y)*scale;
    position.x+=velocity.x*.1;position.y+=velocity.y*.1;
    max_escape=std::max(max_escape,-position.y);
  }
  assert(entered && handed_off && max_escape>7.4);
}

int main()
{
  test_target_retention_and_safety();
  test_obstacle_detour_and_watch_expiry();
  test_handoff_and_watch();
  test_moving_target_and_off_window();
  test_closed_loop_saddle();
  std::cout<<"distance recovery: target retention, safety, handoff and closed-loop escape passed\n";
}
