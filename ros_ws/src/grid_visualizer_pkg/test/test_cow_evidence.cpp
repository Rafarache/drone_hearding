#include "grid_visualizer_pkg/cow_evidence.hpp"
#include <cassert>
#include <limits>

using namespace grid_visualizer_pkg;

EvidenceFrame frame(int64_t stamp, int source=0,
  std::vector<EvidenceDetection> detections={})
{
  EvidenceFrame f;
  f.source=source;
  f.stamp_ns=stamp;
  f.detections=std::move(detections);
  return f;
}

EvidenceDetection cow(double x=5.25, double y=0.25, double low=-0.1, double high=0.1)
{
  return {x,y,true,low,high};
}

CowEvidence grid() {return CowEvidence(-20,-20,40,40,0.5,0,2);}

void apply(CowEvidence& g, EvidenceFrame f)
{
  assert(g.enqueue(std::move(f)));
  g.update();
}

void scoring()
{
  auto g=grid();
  apply(g,frame(1,0,{cow(),cow()})); // duplicate boxes in one cell count once
  assert(g.score(5.25,.25)==150 && g.active_positions().empty());
  apply(g,frame(2,0,{cow()}));
  assert(g.score(5.25,.25)==255 && g.active_positions().size()==1);
  assert(g.active_positions()[0]==std::make_pair(5.25,.25));
  apply(g,frame(3)); assert(g.score(5.25,.25)==225);
  apply(g,frame(4)); assert(g.score(5.25,.25)==195 && g.active_positions().size()==1);
  apply(g,frame(5)); assert(g.score(5.25,.25)==165 && g.active_positions().empty());
  for (int i=6;i<20;++i) {apply(g,frame(i));}
  assert(g.score(5.25,.25)==0);
  apply(g,frame(20,0,{cow()}));
  for (int i=21;i<25;++i) {apply(g,frame(i));}
  assert(g.score(5.25,.25)==30);
  apply(g,frame(25,0,{cow()}));
  assert(g.score(5.25,.25)==180 && g.active_positions().empty());
  assert(!CowEvidence::is_cow(180) && CowEvidence::is_cow(181));
}

void peers_and_order()
{
  for (bool reverse : {false,true}) {
    auto g=grid();
    apply(g,frame(1,0,{cow()}));
    apply(g,frame(2,0,{cow()}));
    for (int i=3;i<10;++i) {apply(g,frame(i));}
    assert(g.score(5.25,.25)==45);
    auto own=frame(10), peer=frame(1,1,{cow()});
    assert(g.enqueue(reverse ? peer : own));
    assert(g.enqueue(reverse ? own : peer));
    g.update();
    assert(g.score(5.25,.25)==195); // would be 165 if own miss was subtracted
    for (int i=0;i<10;++i) {g.update();}
    assert(g.score(5.25,.25)==195); // timer cannot replay +150
    apply(g,frame(2,1)); // peer empty frames cannot clear own map
    assert(g.score(5.25,.25)==195);
    apply(g,frame(11)); // peer protection ended with previous update
    assert(g.score(5.25,.25)==165);
  }
  auto g=grid();
  assert(g.enqueue(frame(1,0,{cow()})));
  assert(g.enqueue(frame(2))); // own hit wins own empty frame in same update
  assert(g.enqueue(frame(1,1,{cow()})));
  g.update();
  assert(g.score(5.25,.25)==255); // own +, empty -, then peer +
}

void shadows_and_cone()
{
  auto g=grid();
  std::vector<EvidenceDetection> points={cow(10.25,.25),cow(10.25,3.25),cow(10.25,1.25),
    cow(5.25,8.75),cow(4.25,.25),cow(3.25,.25),cow(1.25,.25),cow(19.75,.25),cow(5.25,10.25),cow(-5.25,.25)};
  // Seed from a peer with a wide detection range; only subsequent own empty
  // observations may clear cells, so seeding cannot clear other test cells.
  auto seed=frame(1,1,points); seed.far=30;
  apply(g,seed); seed.stamp_ns=2; apply(g,seed);
  apply(g,frame(1,0,{cow()}));
  assert(g.score(10.25,.25)==255); // behind cow
  assert(g.score(10.25,1.25)==255); // cell partly intersects box shadow
  assert(g.score(4.25,.25)==255); // conservative body-depth margin
  assert(g.score(3.25,.25)==225); // visible space before cow
  assert(g.score(10.25,3.25)==225); // visible space beside shadow
  assert(g.score(1.25,.25)==255); // inside near limit
  assert(g.score(19.75,.25)==255); // cell partly beyond far limit
  assert(g.score(5.25,10.25)==255); // outside cone
  assert(g.score(5.25,8.75)==255); // partly outside FOV boundary
  assert(g.score(-5.25,.25)==255); // behind camera
  apply(g,frame(2));
  assert(g.score(10.25,.25)==225); // old cow confidence casts no shadow
  assert(g.score(4.25,.25)==225);
  apply(g,frame(3,0,{{0,0,false,-.1,.1}}));
  assert(g.score(10.25,.25)==225); // unknown depth protects bearing, adds no cow
  auto rotated=frame(4); rotated.yaw=std::acos(-1.0)/2;
  apply(g,rotated);
  assert(g.score(10.25,.25)==225 && g.score(5.25,10.25)==225);
  // A peer's box cannot cast an own-camera shadow from another viewpoint.
  auto peer=frame(3,1,{cow(5.25,5.25,-1,1)}); peer.y=5;
  assert(g.enqueue(peer)); assert(g.enqueue(frame(5))); g.update();
  assert(g.score(10.25,.25)==195);
}

void validation_and_clock()
{
  auto g=grid();
  g.advance_time(100);
  apply(g,frame(100,0,{cow(),cow(20,.25),cow(1.25,.25)}));
  assert(g.score(20,.25)==0 && g.score(1.25,.25)==150);
  assert(!g.enqueue(frame(100)) && !g.enqueue(frame(99)));
  assert(!g.enqueue(frame(101,-1)) && !g.enqueue(frame(101,2)));
  auto bad=frame(101); bad.x=std::numeric_limits<double>::quiet_NaN();
  assert(!g.enqueue(bad));
  bad=frame(101); bad.fov=-1; assert(!g.enqueue(bad));
  bad=frame(101,0,{cow()}); bad.detections[0].bearing_max=-1;
  assert(!g.enqueue(bad));
  assert(g.enqueue(frame(101,0,{cow()})));
  g.advance_time(1); // clears scores, pending frames and per-source stamp gates
  g.update();
  assert(g.score(5.25,.25)==0 && g.active_positions().empty());
  apply(g,frame(1,0,{cow()}));
  assert(g.score(5.25,.25)==150);
}

int main()
{
  scoring(); peers_and_order(); shadows_and_cone(); validation_and_clock();
}
