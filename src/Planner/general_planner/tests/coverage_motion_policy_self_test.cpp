#include <general_core/exploration/highspeed/coverage_motion_policy.h>
#include <iostream>
#include <stdexcept>

using Eigen::Vector3d;
using namespace fast_planner;
using namespace fast_planner::coverage_motion;

void require(bool condition, const char *reason) {
  if (!condition) throw std::runtime_error(reason);
}

int main() {
  try {
    CoverageMotionConfig cfg;
    require(!cfg.enabled, "non-house profiles must remain opt-in");
    const double rest = executionTime(15, 0, 0, 5, 3, 3);
    const double forward = executionTime(15, 4, 0, 5, 3, 3);
    const double reverse = executionTime(15, 4, std::acos(-1.0), 5, 3, 3);
    require(forward < rest && reverse > rest, "selection must price momentum and reversal");
    require(executionTime(1, 5, 0, 5, 3, 3) > 5.0 / 3.0,
            "a goal inside stopping distance must not appear instantaneous");
    for (double v : {0., 2., 5., 8.}) for (double d : {0.1, 1., 10., 30.})
      require(std::isfinite(executionTime(d, v, 0.7, 5, 3, 3)) &&
              executionTime(d, v, 0.7, 5, 3, 3) > 0, "invalid motion time");

    const SegmentFree open = [](const Vector3d &, const Vector3d &) { return true; };
    const SegmentFree observed = [](const Vector3d &a,const Vector3d &b) {
      return a.x()<=4.73 && b.x()<=4.73;
    };
    const Path remote{{0,0,1},{3,0,1},{9.72,0,1}};
    const auto advance=observedPrefix(remote,observed);
    require(length(advance)>4.2 && length(advance)<4.5 && pathFree(advance,observed),
            "bag regression: safe local prefix discarded with remote goal");
    require(observedPrefix(remote,[](const Vector3d &,const Vector3d &){return false;}).empty(),
            "unsafe head admitted into executable prefix");
    require(observedPrefix(remote,open).back()==remote.back(),"complete observed route shortened unnecessarily");
    // A wall across x=2, with a doorway only at y>=2. Shortcutting through the
    // wall must fail while a visible path around the doorway is simplified.
    const SegmentFree wall = [](const Vector3d &a, const Vector3d &b) {
      if ((a.x() - 2) * (b.x() - 2) >= 0 || std::abs(b.x() - a.x()) < 1e-9) return true;
      return (a + (b - a) * ((2 - a.x()) / (b.x() - a.x()))).y() >= 2;
    };
    Path detour{{0,0,1}, {0,3,1}, {4,3,1}, {4,0,1}};
    const Path shortened = shortcut(detour, 8, wall);
    require(pathFree(shortened, wall), "shortcut crossed occupied wall");
    require(length(shortened) >= 4.0, "shortcut invalid length");
    Path straight = shortcut(detour, 8, open);
    require(straight.size() == 2 && length(straight) == 4.0, "open-space spur was not removed");
    Path cached{{0,0,1}, {3,0,1}, {6,0,1}};
    require(!reconnect(cached, Vector3d(1,.1,1), .75, open).empty(), "valid route not retained");
    const auto joined = reconnect(cached, Vector3d(1,.1,1), .75, open);
    require(angle(joined[1]-joined[0], Vector3d::UnitX())<.1,
            "lateral tracking error introduced a spurious right-angle turn");
    require(reconnect(cached, Vector3d(1,2,1), .75, open).empty(), "distant route branch reused");
    require(reconnect(cached, Vector3d(1,0,1), .75, wall).empty(), "new obstacle did not invalidate cache");
    Path old_route{{0,0,1}, {0,1,1}, {6,1,1}, {6,0,1}};
    require(!preferRetained(old_route, cached, cfg), "materially shorter route ignored");
    require(preferRetained(cached, Path{{0,0,1},{5.8,0,1}}, cfg), "small route fluctuation changed branch");

    Path transit = cached;
    require(appendContinuation(transit, Vector3d(12,0,1), 26, cfg, open), "forward continuation rejected");
    require(transit[2] == Vector3d(6,0,1) && transit.back() == Vector3d(12,0,1),
            "original observation was replaced");
    transit = cached;
    require(!appendContinuation(transit, Vector3d(0,0,1), 26, cfg, open), "reverse continuation accepted");
    require(!appendContinuation(transit, Vector3d(12,0,1), 7, cfg, open), "insufficient horizon accepted");
    require(!appendContinuation(transit, Vector3d(12,0,1), 26, cfg,
        [](const Vector3d &, const Vector3d &) { return false; }), "unknown continuation accepted");
    require(!sameRegion(Vector3d(0,0,1), Vector3d(0,0,4), cfg), "different floors merged");

    Path doorway{{1,-1,1},{1,0,1}};
    const Path around{{1,0,1},{1,2,1},{3,2,1},{3,0,1}};
    // A smooth entry into a bent route is allowed; direct endpoint connection
    // through the wall is forbidden. Keep the doorway vertices in the seed.

    require(!appendContinuation(doorway,around.back(),26,cfg,wall),"wall shortcut accepted");
    require(appendPathContinuation(doorway,around,26,cfg,wall),"known doorway continuation lost");
    require(pathFree(doorway,wall) && doorway.back()==around.back(),"topology corner erased");
    Path untouched{{1,-1,1},{1,0,1}};const auto original=untouched;
    require(!appendPathContinuation(untouched,around,26,cfg,
        [](const Vector3d &,const Vector3d &){return false;}) && untouched==original,
        "failed continuation partially mutated executable path");
    Path bounded{{0,0,1},{1,0,1}};
    require(appendPathContinuation(bounded,Path{{1,0,1},{8,0,1}},26,cfg,observed) &&
        bounded.back().x()<4.5 && pathFree(bounded,observed),
        "partially observed continuation was discarded or crossed unknown boundary");
    PlanningBudget budget;
    const double initial_lead=budget.lead(.5,.08);budget.observe(.8);
    require(budget.lead(.5,.08)>initial_lead,"measured planning latency ignored");
    require(budget.retry(.2,.3,.08)<=.021,"retry waited through command expiry");
    require(budget.retry(.2,4,.08)==.2,"ample command budget lost retry backoff");
    LivenessMonitor live;
    require(!live.stalled(1,Vector3d::Zero(),100,20),"fresh task already blocked");
    require(live.stalled(22,Vector3d(.1,0,0),102,20),"stationary ID churn hid terminal stall");
    require(!live.stalled(23,Vector3d(.1,0,0),110,20),"measured map gain ignored");
    require(!live.stalled(44,Vector3d(1,0,0),110,20),"measured motion ignored");

    PassageMonitor monitor;
    monitor.active = true; monitor.goal = Vector3d(1,0,1); monitor.radius = .35;
    monitor.goal = Vector3d(.25,0,1); monitor.radius = .2;
    monitor.observe(Vector3d(0,0,1), 1, 5);
    monitor.observe(Vector3d(.5,0,1), 1.1, 5);
    require(monitor.passed, "measured high-speed passage was missed");
    monitor = {}; monitor.active = true; monitor.goal = Vector3d(1,0,1);
    monitor.observe(Vector3d(0,0,1), 1, 5);
    monitor.observe(Vector3d(2,0,1), 3, 5);
    require(!monitor.passed, "stale odometry chord completed observation");
    monitor = {}; monitor.active = true; monitor.goal = Vector3d(1,0,1);
    monitor.observe(Vector3d(0,0,1), 1, 5);
    monitor.observe(Vector3d(2,0,1), 1.01, 5);
    require(!monitor.passed, "odometry jump completed observation");
    std::cout << "coverage motion policy: momentum, route safety, continuation, passage PASS\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
