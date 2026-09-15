#include <general_core/exploration/highspeed/coverage_route_policy.h>
#include <general_core/exploration/exploration_utils/coverage_guidance/coverage_component_identity.h>
#include <general_core/exploration/exploration_utils/coverage_guidance/coverage_candidate_window.h>
#include <iostream>
#include <stdexcept>
using namespace fast_planner;
using namespace fast_planner::coverage_route;
void require(bool value,const char *message) {if(!value) throw std::runtime_error(message);}
int main() {
  try {
    CoverageComponentIdentity identity;
    require(coverageCandidateWindow({0,1,2,3,4,5},{false,false,false,false,true,true},4)==
        std::vector<int>({0,1,4,5}),"distance truncation dropped coverage intention");
    require(coverageCandidateWindow({0,1,2},{true,false,false},4)==std::vector<int>({0,1,2}),
        "sparse preferred pool duplicated a task or lost a local fallback");
    auto a=identity.update({0,0,0,1,1},{1,1});
    auto b=identity.update({1,1,1,0,0},{1,1});
    require(a[0]==b[1] && a[1]==b[0],"component renumbering changed identity");
    auto c=identity.update({0,0,1,2,2},{1,1,1});
    require(c[0]==a[0] && c[1]!=a[0] && c[2]==a[1],"split identity not inherited by largest overlap");
    auto d=identity.update({0,0,0,0,0},{1});
    require(d[0]==std::min(c[0],c[2]),"merge did not preserve deterministic predecessor");
    auto e=identity.update({0,0,0,0,0},{0});
    require(e[0]!=d[0],"free/unknown transition reused semantic identity");
    Problem p; p.groups=2;p.anchors=2;p.max_speed=3;p.acceleration=2;p.yaw_rate=2;
    p.nodes.resize(5);
    p.nodes[1].position={2,0,0};p.nodes[1].group=0;
    p.nodes[2].position={3,0,0};p.nodes[2].group=1;
    p.nodes[3].position={5,0,0};p.nodes[3].anchor=0;
    p.nodes[4].position={8,0,0};p.nodes[4].anchor=1;
    for(int k=0;k<32;++k) {p.nodes[1].visible.push_back(k);p.nodes[2].visible.push_back(k+32);}
    auto connect=[&](){p.edges.assign(p.nodes.size(),std::vector<Edge>(p.nodes.size()));
      for(int i=0;i<(int)p.nodes.size();++i) for(int j=1;j<(int)p.nodes.size();++j) if(i!=j) {
        auto &edge=p.edges[i][j];edge.valid=true;edge.executable=i<3&&j<3;
        edge.first=p.nodes[j].position-p.nodes[i].position;edge.last=edge.first;edge.length=edge.first.norm();
      }};
    connect();Config cfg;cfg.solve_ms=100;cfg.beam_width=64;
    auto route=solve(p,cfg);
    require(route.valid && route.complete && route.prefix.size()==2,"complete anchored route dropped a local obligation");
    auto first=std::find(route.route.begin(),route.route.end(),3),second=std::find(route.route.begin(),route.route.end(),4);
    require(first<second && second!=route.route.end(),"anchor precedence broken");
    for(auto index:route.prefix) require(index<3,"symbolic anchor became executable");
    p.mandatory=2;p.edges[0][2].valid=false;p.edges[1][2].valid=false;
    require(!solve(p,cfg).valid,"unreachable mandatory task silently discarded");
    connect();route=solve(p,cfg);
    require(std::find(route.prefix.begin(),route.prefix.end(),2)!=route.prefix.end(),"overdue task deferred again");
    p.nodes[2].group=0;p.groups=1;p.mandatory=1;route=solve(p,cfg);
    require(route.valid && route.prefix.size()==1,"mutually exclusive views visited twice");
    p.committed_first=2;route=solve(p,cfg);
    require(route.valid && route.prefix.front()==2,"valid committed prefix replaced");
    p.edges[0][2].executable=false;require(!solve(p,cfg).valid,"hypothetical edge executed");
    p.edges[0][2].executable=true;p.edges[0][2].length=std::numeric_limits<double>::quiet_NaN();
    require(!solve(p,cfg).valid,"non-finite edge accepted");
    // The same local tasks must change order when the coverage exit changes;
    // this checks structural intention rather than a decorative rank field.
    p.committed_first=-1;p.groups=2;p.mandatory=0;
    p.nodes[1].group=0;p.nodes[2].group=1;
    p.nodes[1].position={-3,0,0};p.nodes[2].position={3,0,0};
    p.nodes[2].visible=p.nodes[1].visible;
    p.nodes[3].position={8,0,0};p.nodes[4].position={11,0,0};connect();
    route=solve(p,cfg);require(route.valid && route.prefix.front()==1 && route.complete,"right exit should clear left room before departing right");
    p.nodes[3].position.x()=-8;p.nodes[4].position.x()=-11;connect();
    route=solve(p,cfg);require(route.valid && route.prefix.front()==2 && route.complete,"left exit should clear right room before departing left");
    // The exit influences order even with no artificial coverage gain reward.
    for (auto &node:p.nodes) node.visible.clear();
    route=solve(p,cfg);require(route.complete && route.prefix.front()==2,"zero reward lost room obligation");
    cfg.max_prefix_time=.01;route=solve(p,cfg);
    require(route.complete && route.prefix.size()==1 && route.deferred.size()==1,
        "short execution horizon erased long-horizon observation work");
    cfg.max_prefix_time=10;cfg.solve_ms=.01;route=solve(p,cfg);
    require(route.valid && route.complete,"small solver budget lost feasible incumbent");
    cfg.solve_ms=100;
    p.edges[1][2].valid=false;p.edges[2][1].valid=false;
    for (int anchor:{3,4}) for (int task:{1,2}) p.edges[anchor][task].valid=false;
    route=solve(p,cfg);
    require(route.valid && !route.complete && route.deferred.size()==1,
        "disconnected task was silently reported as covered");
    connect();
    p.preferred_first=1;cfg.switch_margin=.01;route=solve(p,cfg);
    require(route.prefix.front()==2,"route continuity hid material improvement");
    p.preferred_first=-1;
    // An entry on the route may precede another local task. Requiring every
    // frontier before CP postpones region transitions and causes room sweeps
    // to dominate new-space exploration. The deferred task must still exist.
    Problem mixed;mixed.groups=2;mixed.anchors=1;mixed.nodes.resize(4);
    mixed.nodes[1].group=0;mixed.nodes[2].group=1;mixed.nodes[3].anchor=0;
    mixed.edges.assign(4,std::vector<Edge>(4));
    auto arc=[&](int from,int to,double distance,bool executable) {
      auto &edge=mixed.edges[from][to];edge.valid=true;edge.executable=executable;edge.length=distance;
    };
    arc(0,1,1,true);arc(0,2,30,true);arc(1,2,10,true);arc(2,1,10,true);
    arc(1,3,2,false);arc(3,2,3,false);arc(2,3,3,false);arc(3,1,2,false);
    auto joint=solve(mixed,cfg);
    require(joint.complete && joint.route==std::vector<int>({1,3,2}) &&
        joint.prefix==std::vector<int>({1}) && joint.deferred==std::vector<int>({1}),
        "CP entry cannot precede a deferred frontier in the same optimization");
    mixed.entry_groups=2;joint=solve(mixed,cfg);
    require(joint.complete && joint.prefix.front()==2,"CP active-region entry order was discarded");
    mixed.edges[0][2].executable=false;
    require(!solve(mixed,cfg).valid,"unsafe active-region entry became a command");
    mixed.edges[0][2].executable=true;mixed.entry_groups=0;
    mixed.mandatory=2;joint=solve(mixed,cfg);
    require(joint.complete && std::find(joint.route.begin(),joint.route.end(),2)<
        std::find(joint.route.begin(),joint.route.end(),3),"overdue observation starved behind CP repeatedly");
    require(!Config{}.enabled,"new policy enabled for default task modes");
    ProgressMonitor progress;
    require(!progress.observe(1,4,0,3),"first task selection already stalled");
    require(!progress.observe(3,3,0,3),"actual approach did not reset watchdog");
    require(!progress.observe(5,2.95,.01,3),"watchdog expired before deadline");
    require(progress.observe(7,2.9,.02,3),"viewpoint jitter hid a stalled observation");
    require(!progress.observe(8,3,.2,3),"new sensor evidence did not reset watchdog");
    require(!progress.observe(20,6,.2,3),"new execution opportunity inherited expired timer");
    p.nodes[1].stop=true;p.mandatory=2;
    route=solve(p,cfg);
    require(route.valid && route.complete,"stopped prefix deleted a future observation obligation");
    bool stopped=false;
    for (int index:route.prefix) {require(!stopped,"executable prefix continues after recovery stop");stopped=p.nodes[index].stop;}
    std::cout<<"coverage route policy self-test passed\n";
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
