#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace general_planner {
// Lazy graph: one vertex per maximal vertical traversable run. No free-voxel
// materialization. A missing graph route is NOT proof of global unreachability:
// bounds, budgets, conservative portals and live updates can all remove routes.
class BdmIntervalSearch {
public:
    using Index = Eigen::Vector3i;
    using Runs = std::vector<std::pair<int,int>>;
    using ColumnQuery = std::function<bool(int,int,int,int,Runs&)>;
    struct Result {
        enum Status { FOUND, DISCONNECTED, BUDGET, UNAVAILABLE } status{UNAVAILABLE};
        std::vector<Index> path;
        bool reached_goal{false};
        std::size_t expanded{0}, columns{0}, nodes{0};
    };
    static Result search(const Index &start, const Index &goal,
                         const Index &lo, const Index &hi, bool diagonal,
                         const ColumnQuery &query, double budget_s,
                         std::size_t max_expanded = 6000,
                         double horizon = std::numeric_limits<double>::infinity()) {
        Result result;
        const auto begin = std::chrono::steady_clock::now();
        struct Node {
            int x,y,lo,hi,z,parent{-1},portal_z{0};
            double g{std::numeric_limits<double>::infinity()};
            bool closed{false};
        };
        struct Entry { double f,g; int id; bool operator<(const Entry &b) const {
            return f == b.f ? id > b.id : f > b.f;
        }};
        std::vector<Node> nodes;
        std::unordered_map<std::uint64_t,std::vector<int>> columns;
        bool available=true;
        auto column = [&](int x,int y) -> std::vector<int> {
            if (x<lo.x() || x>hi.x() || y<lo.y() || y>hi.y()) return {};
            const auto key=(std::uint64_t(std::uint32_t(x))<<32)|std::uint32_t(y);
            const auto found=columns.find(key);
            if(found!=columns.end()) return found->second;
            Runs runs;
            if(!query(x,y,lo.z(),hi.z(),runs)) { available=false; return {}; }
            std::vector<int> ids;
            for(const auto &run:runs) {
                if(run.first>run.second || run.first<lo.z() || run.second>hi.z()) {
                    available=false; return {};
                }
                ids.push_back(int(nodes.size()));
                nodes.push_back({x,y,run.first,run.second,run.first});
            }
            columns.emplace(key,ids);
            return ids;
        };
        auto containing = [&](const Index &p) {
            for(int id:column(p.x(),p.y()))
                if(p.z()>=nodes[id].lo && p.z()<=nodes[id].hi) return id;
            return -1;
        };
        const int source=containing(start), target=containing(goal);
        auto finish = [&](Result::Status status) {
            result.status=status; result.columns=columns.size();result.nodes=nodes.size();return result;
        };
        if(!available) return finish(Result::UNAVAILABLE);
        if(source<0 || (target<0 && !std::isfinite(horizon))) return finish(Result::DISCONNECTED);
        auto heuristic = [&](const Node &n) {
            const double dz=std::max({n.lo-goal.z(),goal.z()-n.hi,0});
            return std::sqrt(double(n.x-goal.x())*(n.x-goal.x())+
                             double(n.y-goal.y())*(n.y-goal.y())+dz*dz);
        };
        nodes[source].g=0;nodes[source].z=start.z();
        std::priority_queue<Entry> open;
        open.push({heuristic(nodes[source]),0,source});
        while(!open.empty()) {
            if(result.expanded>=max_expanded || (budget_s>0 &&
               std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>budget_s))
                return finish(Result::BUDGET);
            const auto entry=open.top();open.pop();
            if(nodes[entry.id].closed || entry.g!=nodes[entry.id].g) continue;
            nodes[entry.id].closed=true;
            const Node current=nodes[entry.id]; // column insertion may reallocate nodes
            ++result.expanded;
            if(entry.id==target || current.g>horizon) {
                result.reached_goal=entry.id==target;
                const Index endpoint=result.reached_goal ? goal : Index(current.x,current.y,current.z);
                std::vector<int> chain;
                for(int id=entry.id;id>=0;id=nodes[id].parent) chain.push_back(id);
                std::reverse(chain.begin(),chain.end());
                result.path.push_back(start);
                for(std::size_t k=1;k<chain.size();++k) {
                    const Node &p=nodes[chain[k-1]], &n=nodes[chain[k]];
                    const Index portal(p.x,p.y,n.portal_z), next(n.x,n.y,n.z);
                    if(result.path.back()!=portal) result.path.push_back(portal);
                    if(result.path.back()!=next) result.path.push_back(next);
                }
                if(result.path.back()!=endpoint) result.path.push_back(endpoint);
                return finish(Result::FOUND);
            }
            for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
                if((dx==0 && dy==0) || (!diagonal && dx!=0 && dy!=0)) continue;
                for(int id:column(current.x+dx,current.y+dy)) {
                    if(nodes[id].closed) continue;
                    const Node next=nodes[id];
                    // Overlapping runs give a horizontal portal. Disjoint
                    // z-diagonal connections are deliberately left to voxel A*.
                    const int low=std::max(current.lo,next.lo), high=std::min(current.hi,next.hi);
                    if(low>high) continue;
                    const int z=std::clamp(current.z,low,high);
                    if(dx!=0 && dy!=0) {
                        auto containsZ = [&](int x,int y) {
                            for(int side:column(x,y))
                                if(z>=nodes[side].lo && z<=nodes[side].hi) return true;
                            return false;
                        };
                        // Do not propose diagonal corner cutting; original line
                        // validation checks intermediate ray cells as well.
                        if(!containsZ(current.x+dx,current.y) || !containsZ(current.x,current.y+dy)) continue;
                    }
                    Node &n=nodes[id];
                    const double g=current.g+std::abs(z-current.z)+std::hypot(dx,dy);
                    if(g>=n.g) continue;
                    n.g=g;n.z=z;n.portal_z=z;n.parent=entry.id;
                    open.push({g+heuristic(n),g,id});
                }
                if(!available) return finish(Result::UNAVAILABLE);
            }
        }
        return finish(Result::DISCONNECTED);
    }
};
} // namespace general_planner
