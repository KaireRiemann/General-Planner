#include <general_core/exploration/highspeed/epicon_execution_policy.h>
#include <iostream>
#include <stdexcept>

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  using fast_planner::EpiconFinishVerifier;
  try {
    fast_planner::EpiconProgressWatchdog progress;
    progress.reset(100.0);
    for (int n=1;n<=50;++n) progress.observe(100+n,false);
    require(progress.idleSeconds(150)==50,"recovery or goal reselection reset the progress timeout");
    progress.observe(151,true);
    require(progress.idleSeconds(153)==2,"approaching an exploration goal did not count as progress");
    const std::vector<Eigen::Vector3f> around_wall{{0,0,1},{0,-2,1},{3,-2,1},{3,0,1},{1,0,1}};
    const Eigen::Vector3f before(0,0,1),after(0,-1,1);
    require((after-around_wall.back()).norm()>(before-around_wall.back()).norm(),"test does not move away from goal");
    require(fast_planner::epiconRemainingRoute(after,around_wall)<
            fast_planner::epiconRemainingRoute(before,around_wall)-.9,"safe wall detour was not progress");
    require(fast_planner::epiconRemainingRoute(around_wall.back(),around_wall)<1e-6,"route endpoint has remaining distance");
    fast_planner::EpiconGoalWindow window;
    window.select({1,0,1},139.417);
    require(!window.expired(139.420,8),"new retreat inherited old goal timeout");
    window.select({1.1f,0,1},145);
    require(window.expired(148,8),"same goal replanning renewed execution window");
    window.select({3,0,1},148);
    require(!window.expired(149,8),"different goal did not receive execution window");
    window.progress(154);
    require(!window.expired(160,8),"route progress did not renew goal window");
    require(progress.idleSeconds(191)==40,"goal windows reset mission timeout");
    window.reset();
    require(!window.expired(200,8),"new task inherited goal window");
    window.select({3,0,1},200);
    require(!window.expired(201,8),"explicit retry of same goal inherited old attempt age");
    require(progress.idleSeconds(201)==50,"explicit retry replenished mission budget");
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
