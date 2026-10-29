#include "grid_visualizer_pkg/stall_recovery.hpp"
#include <cassert>

int main()
{
  grid_visualizer_pkg::StallRecovery tracker;
  for (int t=0;t<6;++t) {assert(!tracker.update(t,0,0,true,6,.4));}
  assert(tracker.update(6,0,0,true,6,.4));
  // The detector does not decide that one metre of displacement ends recovery.
  tracker.reset();
  for (int t=0;t<=12;++t) {assert(!tracker.update(t,t*.2,0,true,6,.4));}
  tracker.reset();
  for (int t=0;t<=6;++t) {
    assert(tracker.update(t,t%2 ? .15:-.15,0,true,6,.4)==(t==6));
  }
  tracker.reset();
  // Larger oscillations are outside the old 0.4 m confinement test.
  for (int t=0;t<6;++t) {assert(!tracker.update(t,t%2 ? .7:-.7,0,true,6,.4));}
  assert(tracker.update(6,-.7,0,true,6,.4));
  assert(!tracker.update(7,0,0,false,6,.4));
  assert(!tracker.update(10,0,0,true,6,.4));
  assert(!tracker.update(1,0,0,true,6,.4));
  assert(!tracker.update(1,0,0,true,6,.4));
  tracker.reset();
  // One deliberate U-turn is not repeated oscillation.
  for (int t=0;t<=6;++t) {assert(!tracker.update(t,t<3 ? t*.3:(6-t)*.3,0,true,6,.4));}
}
