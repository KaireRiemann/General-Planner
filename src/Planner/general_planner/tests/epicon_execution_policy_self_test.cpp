#include <general_core/exploration/highspeed/epicon_execution_policy.h>
#include <iostream>
#include <stdexcept>

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  using fast_planner::EpiconFinishVerifier;
  try {
    EpiconFinishVerifier gate;
    // Recorded failure: empty selection 0.418 s after committing a trajectory
    // with 5.466 s remaining. Repeated timer callbacks must not stop it.
    for (int n=0;n<20;++n)
      require(!gate.observe(100+n*.1,false,true,true,1,99,4,1.5),"unfinished motion completed");
    require(!gate.observe(103,true,true,true,2,102.9,4,1.5),"pre-settle scan counted");
    for (int n=0;n<20;++n)
      require(!gate.observe(104+n*.1,true,true,true,3,103.5,4,1.5),"duplicate scan completed");
    require(gate.count()==1,"same revision counted twice");
    require(!gate.observe(107,true,true,true,4,106.9,4,1.5),"insufficient observations completed");
    require(!gate.observe(108,true,true,false,5,107.9,4,1.5),"new frontier did not cancel finish");
    require(gate.count()==0,"candidate recovery retained evidence");
    require(!gate.observe(109,true,false,true,6,108.9,4,1.5),"stale sensor completed");
    for(int n=0;n<4;++n)
      require(!gate.observe(110+n*.2,true,true,true,7+n,110+n*.2-.05,4,1.5),"minimum duration bypassed");
    require(gate.observe(113,true,true,true,11,112.9,4,1.5),"independent settled audits did not complete");
    gate.reset();
    require(!gate.observe(114,true,true,true,12,113.9,4,1.5),"new task inherited completion");
    std::cout << "EPICON_EXECUTION_POLICY PASS: ongoing motion, independent frames, freshness, recovery, duration, reset\n";
    return 0;
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n';return 1; }
}
