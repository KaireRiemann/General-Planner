#include <map_manager/bdm_interval_search.hpp>
#include <iostream>
#include <random>
#include <queue>
#include <stdexcept>
using Search=general_planner::BdmIntervalSearch;
using I=Search::Index;
int main() {
    std::mt19937 random(42);
    constexpr int X=13,Y=11,Z=9;
    auto hash=[](int x,int y,int z){return (x*Y+y)*Z+z;};
    std::size_t tested=0;
    for(int trial=0;trial<500;++trial) {
        std::vector<bool> blocked(X*Y*Z);
        for(int i=0;i<X*Y*Z;++i) blocked[i]=random()%100<30;
        I start(random()%X,random()%Y,random()%Z),goal(random()%X,random()%Y,random()%Z);
        blocked[hash(start.x(),start.y(),start.z())]=false;
        blocked[hash(goal.x(),goal.y(),goal.z())]=false;
        auto query=[&](int x,int y,int lo,int hi,Search::Runs &runs) {
            int begin=-1;
            for(int z=lo;z<=hi+1;++z) {
                const bool free=z<=hi && !blocked[hash(x,y,z)];
                if(free && begin<0) begin=z;
                if(!free && begin>=0) {runs.emplace_back(begin,z-1);begin=-1;}
            }
            return true;
        };
        auto result=Search::search(start,goal,I::Zero(),I(X-1,Y-1,Z-1),false,query,0,100000);
        std::vector<bool> seen(blocked.size());std::queue<I> queue;queue.push(start);
        seen[hash(start.x(),start.y(),start.z())]=true;
        while(!queue.empty()) {
            I p=queue.front();queue.pop();
            for(int axis=0;axis<3;++axis) for(int sign:{-1,1}) {
                I n=p;n[axis]+=sign;
                if((n.array()<0).any() || n.x()>=X || n.y()>=Y || n.z()>=Z) continue;
                int h=hash(n.x(),n.y(),n.z());if(blocked[h]||seen[h]) continue;
                seen[h]=true;queue.push(n);
            }
        }
        if((result.status==Search::Result::FOUND)!=seen[hash(goal.x(),goal.y(),goal.z())])
            throw std::runtime_error("six-connected reachability mismatch");
        for(std::size_t k=1;k<result.path.size();++k) {
            I p=result.path[k-1],end=result.path[k];
            if((p-end).array().count()>1) throw std::runtime_error("invalid portal");
            while(p!=end) {
                if(blocked[hash(p.x(),p.y(),p.z())]) throw std::runtime_error("occupied path");
                for(int axis=0;axis<3;++axis) if(p[axis]!=end[axis]) {p[axis]+=p[axis]<end[axis]?1:-1;break;}
            }
        }
        ++tested;
    }
    auto empty=[](int,int,int lo,int hi,Search::Runs&r){r.emplace_back(lo,hi);return true;};
    auto open=Search::search(I(0,0,-50),I(80,80,50),I(-1,-1,-100),I(81,81,100),true,empty,0);
    if(open.status!=Search::Result::FOUND || open.expanded>200) throw std::runtime_error("open volume compression failed");
    auto limited=Search::search(I::Zero(),I(10,10,0),I::Zero(),I(20,20,5),true,empty,0,1);
    if(limited.status!=Search::Result::BUDGET) throw std::runtime_error("budget failure");
    auto horizon=Search::search(I::Zero(),I(80,80,0),I::Zero(),I(81,81,5),true,empty,0,6000,10);
    if(horizon.status!=Search::Result::FOUND || horizon.reached_goal || horizon.expanded>20)
        throw std::runtime_error("horizon termination failed");
    auto corner=[](int x,int y,int,int,Search::Runs&r) {
        if(x==y) r.emplace_back(0,0);
        return true;
    };
    if(Search::search(I::Zero(),I(1,1,0),I::Zero(),I(1,1,0),true,corner,0).status==Search::Result::FOUND)
        throw std::runtime_error("diagonal corner cut");
    auto absent=[](int,int,int,int,Search::Runs&){return false;};
    if(Search::search(I::Zero(),I::Ones(),I::Zero(),I::Ones(),false,absent,0).status!=Search::Result::UNAVAILABLE)
        throw std::runtime_error("unavailable failure");
    std::cout<<"bdm_interval_search_self_test PASS randomized="<<tested<<" open_expanded="<<open.expanded<<"\n";
}
