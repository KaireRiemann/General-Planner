#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <general_core/exploration/highspeed/expl_data.h>
#include <map>
#include <sstream>
#include <queue>

namespace fast_planner {

void FastExplorationManager::notifyCoverageRoutePassage(std::uint64_t identity) {
  if (!coverageRouteEnabled()) return;
  // The submitted gate owns identity even if a later planning attempt has
  // replaced the candidate window. Nearby tasks are not passage evidence.
  const auto it=coverage_route_tasks_.find(identity);
  if (it==coverage_route_tasks_.end() || it->second.verifying) return;
  const auto snapshot=coverage_guidance_ ? coverage_guidance_->latestUsablePlan() : CoveragePlan::Ptr{};
  it->second.verifying=true;
  it->second.deferred=0;
  it->second.evidence_version=snapshot ? snapshot->frontier_version : 0;
  ROS_INFO_STREAM("[coverage route] VERIFYING task=" << identity
                  << " frontier_version=" << it->second.evidence_version);
}

bool FastExplorationManager::planCoverageRoute(const CoveragePlan::Ptr &snapshot,
    const vector<TopoNode::Ptr> &candidates, const vector<EdgeSafetyCost> &start_edges,
    const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, std::vector<int> &prefix) {
  using namespace coverage_route;
  prefix.clear();
  const auto previous_observations = coverage_route_observations_;
  coverage_route_observations_.clear();
  coverage_route_exit_path_.clear();
  coverage_route_stalled_=false;
  if (candidates.empty() || candidates.size()!=start_edges.size()) return false;
  const auto &cfg=coverage_route_config_;
  const double now=ros::Time::now().toSec();
  const auto started=std::chrono::steady_clock::now();
  auto elapsed=[&](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();};

  // A pass is not an observation. New measured boundary evidence may also
  // complete a task before arrival (e.g. observed from another valid pose).
  for (auto it=coverage_route_tasks_.begin();it!=coverage_route_tasks_.end();) {
    const bool absent=snapshot && !snapshot->cluster_region.count(it->second.cluster);
    if (snapshot && snapshot->frontier_version>it->second.evidence_version &&
        frontier_manager_ptr_->observedBoundaryFraction(it->second.cells)>=.7) {
      ROS_INFO_STREAM("[coverage route] OBSERVED task=" << it->first
                      << " frontier_version=" << snapshot->frontier_version);
      it=coverage_route_tasks_.erase(it);
    } else if (absent && now-it->second.last_seen>120.0) {
      // Removed from the active task cache, never counted toward coverage.
      it=coverage_route_tasks_.erase(it);
    } else ++it;
  }

  Problem problem;
  Node origin; origin.position=pos; origin.yaw=planner_manager_->local_data_.curr_yaw_;
  problem.speed=vel.norm(); problem.direction=vel;
  Eigen::Vector3d switch_pos,switch_vel; double switch_yaw=origin.yaw;
  if (planner_manager_->getCommittedReplanHeadState(switch_pos,switch_vel,switch_yaw)) {
    origin.position=switch_pos; origin.yaw=switch_yaw;
    problem.speed=switch_vel.norm(); problem.direction=switch_vel;
  }
  problem.nodes.push_back(origin);
  problem.max_speed=ep_->v_max_; problem.acceleration=std::min(ep_->a_avg_,planner_manager_->gcopter_config_->brakeAccel);
  problem.yaw_rate=ep_->yaw_v_max_;

  std::map<int,int> cluster_group;
  std::map<std::uint64_t,int> coverage_group;
  std::vector<std::uint64_t> group_ids;
  std::unordered_set<std::uint64_t> associated;
  std::vector<int> group_sizes;
  // Deterministic input order is already sorted by reachable travel cost.
  for (int i=0;i<static_cast<int>(candidates.size());++i) {
    const auto &vp=candidates[i];
    int group=-1;
    if (vp->is_coverage_target_) {
      auto it=coverage_group.find(0);
      if (it!=coverage_group.end()) group=it->second;
    } else {
      auto it=cluster_group.find(vp->frontier_cluster_id_);
      if (it!=cluster_group.end()) group=it->second;
    }
    if (group<0) {
      if (problem.groups>=cfg.max_tasks) continue;
      group=problem.groups++;
      std::uint64_t identity=0;
      const auto region_it=snapshot ? snapshot->cluster_region.find(vp->frontier_cluster_id_) :
          std::unordered_map<int,std::uint64_t>::const_iterator{};
      std::uint64_t region=snapshot && region_it!=snapshot->cluster_region.end() ? region_it->second : 0;
      if (!vp->is_coverage_target_) {
        double best=0.0;
        for (const auto &entry:coverage_route_tasks_) {
          if (associated.count(entry.first) || entry.second.cluster<0) continue;
          const auto &old=entry.second;
          if (std::abs(old.position.z()-vp->center_.z())>1.0) continue;
          int overlap=0;
          for (auto cell:vp->coverage_visible_cells_) overlap+=std::binary_search(old.visible.begin(),old.visible.end(),cell);
          const double ratio=static_cast<double>(overlap)/std::max<std::size_t>(1,std::min(old.visible.size(),vp->coverage_visible_cells_.size()));
          const double distance=(old.position-vp->center_.cast<double>()).norm();
          const double score=ratio>=.25 ? ratio+1.0 :
              (region==old.region && distance<.75 ? 1.0-distance : 0.0);
          if (score>best) {best=score;identity=entry.first;}
        }
        if (!identity) identity=coverage_next_task_id_++;
        cluster_group[vp->frontier_cluster_id_]=group;
      } else {
        coverage_group[0]=group;
        // Recovery IDs belong to another namespace; do not overwrite a
        // frontier observation task with the same numeric region identity.
        for (const auto &task:coverage_route_tasks_)
          if (task.second.cluster<0 && (task.second.position-vp->center_.cast<double>()).norm()<.1) {
            identity=task.first;break;
          }
        if (!identity) identity=coverage_next_task_id_++;
      }
      associated.insert(identity);
      auto &task=coverage_route_tasks_[identity];
      // A frontier update can precede the next asynchronous CP snapshot.
      // Preserve the region attached to the matched measured boundary task.
      if (!region && snapshot && std::any_of(snapshot->regions.begin(),snapshot->regions.end(),
          [&](const CoverageRegion &known) {return known.identity==task.region && known.state==CoverageVoxelState::KNOWN_FREE;}))
        region=task.region;
      task.cluster=vp->frontier_cluster_id_; task.position=vp->center_.cast<double>();
      task.region=region; task.visible=vp->coverage_visible_cells_; task.last_seen=now;
      if (task.cells.empty()) {
        task.cells=vp->coverage_visible_indices_;
        task.evidence_version=snapshot ? snapshot->frontier_version : 0;
      }
      group_ids.push_back(identity);group_sizes.push_back(0);
    }
    if (group_sizes[group]++ >= (vp->is_coverage_target_ ? cfg.max_tasks : cfg.alternatives)) continue;
    Node node;node.position=vp->center_.cast<double>();node.yaw=vp->yaw_;
    node.group=group;node.source=i;node.identity=group_ids[group];
    node.stop=vp->is_coverage_target_;
    node.region=coverage_route_tasks_[node.identity].region;
    node.visible=vp->coverage_visible_cells_;
    // Unknown intention already enters through the ordered region suffix.
    // Do not fabricate visible cells or reward an unmeasured approach twice.
    problem.nodes.push_back(std::move(node));
  }
  if (!problem.groups) return false;
  const int observation_end=problem.nodes.size();

  // CP includes active free regions as well as unknown hypotheses. Keep its
  // next represented active region as the entry stage; optimizing only toward
  // the first unknown dropped this part of FALCON's long-horizon intention.
  // This selects a region of alternative tasks, never one preselected point.
  if (snapshot && coverage_group.empty()) {
    for (const auto &target:snapshot->ordered_targets) {
      if (target.type!=CoverageTargetType::ACTIVE_FREE) continue;
      for (int i=1;i<observation_end;++i)
        if (problem.nodes[i].region==target.region_id)
          problem.entry_groups|=1U<<problem.nodes[i].group;
      if (problem.entry_groups) break;
    }
  }

  // The CP contributes the next reachable unknown-region ENTRY, not an
  // unknown voxel command nor a second set of mandatory local observations.
  std::vector<CoverageTarget> anchors;
  auto addAnchor = [&](const CoverageTarget &target) {
    if (target.type != CoverageTargetType::REACHABLE_UNKNOWN || !target.has_approach ||
        !target.approach_position.allFinite() ||
        (target.approach_position-origin.position).norm()<1.0 ||
        coverageRecoveryExhausted(target) || coverageRecoveryDeferred(target,ros::Time::now())) return;
    if (std::none_of(anchors.begin(),anchors.end(),[&](const CoverageTarget &a) {
          return a.stable_id==target.stable_id; })) anchors.push_back(target);
  };
  if (snapshot && snapshot->valid && coverage_group.empty()) {
    if (now-coverage_intention_since_<cfg.intention_hold)
      for (auto id:coverage_intention_ids_) for (const auto &target:snapshot->ordered_targets)
        if (target.stable_id==id) {addAnchor(target);break;}
    for (const auto &target:snapshot->ordered_targets) {
      if (anchors.size()>=static_cast<std::size_t>(std::max(1,cfg.anchors))) break;
      addAnchor(target);
    }
  }
  problem.edges.resize(observation_end,std::vector<Edge>(observation_end));
  std::vector<std::vector<coverage_motion::Path>> paths(observation_end,
      std::vector<coverage_motion::Path>(observation_end));
  auto copyEdge=[&](const EdgeSafetyCost &source,int a,int b,bool executable) {
    Edge edge;
    edge.valid=std::isfinite(source.total_cost) && source.total_cost<1999 && source.path_length>0;
    edge.executable=edge.valid && executable;
    edge.length=source.path_length;edge.first=source.first_tangent;edge.last=source.last_tangent;
    edge.speed_limit=source.speed_limit;
    if (edge.first.norm()<1e-5 && source.path.size()>1)
      edge.first=(source.path[1]-source.path[0]).cast<double>();
    if (edge.last.norm()<1e-5 && source.path.size()>1)
      edge.last=(source.path.back()-source.path[source.path.size()-2]).cast<double>();
    edge.bend=std::min(1.5,source.turn_angle);
    problem.edges[a][b]=edge;
    for (const auto &point:source.path) paths[a][b].push_back(point.cast<double>());
  };
  const coverage_motion::SegmentFree observed_free = [&](const Eigen::Vector3d &a,const Eigen::Vector3d &b) {
    const int steps=std::max(1,static_cast<int>(std::ceil((b-a).norm()/.1)));
    for (int k=0;k<=steps;++k)
      if (!planner_manager_->isObservedLocalKnownFree(a+(b-a)*(static_cast<double>(k)/steps))) return false;
    return true;
  };
  for (int i=1;i<observation_end;++i) {
    auto edge=start_edges[problem.nodes[i].source];
    // Align cost and retained geometry to the same future switch state. Only
    // the short join is certified here; the adapter validates the full prefix.
    if (edge.path.size()>1 && (edge.path.front().cast<double>()-origin.position).norm()>.02) {
      double best=2.0;std::size_t join=edge.path.size();
      for (std::size_t k=1;k<edge.path.size();++k) {
        const double d=coverage_motion::pointSegmentDistance(origin.position,
            edge.path[k-1].cast<double>(),edge.path[k].cast<double>());
        if (d<best && observed_free(origin.position,edge.path[k].cast<double>())) {best=d;join=k;}
      }
      if (join<edge.path.size()) {
        std::vector<Eigen::Vector3f> adjusted{origin.position.cast<float>()};
        adjusted.insert(adjusted.end(),edge.path.begin()+join,edge.path.end());
        edge=planner_manager_->estimateHighSpeedEdgeCost(adjusted,problem.direction,
            origin.yaw,problem.nodes[i].yaw);
        edge.path=std::move(adjusted);
      } else { edge.total_cost=2000; }
    }
    const bool forward=problem.speed<=.5 || edge.initial_heading_delta<=
        planner_manager_->gcopter_config_->reorientationHeadingAngle;
    copyEdge(edge,0,i,forward);
    if (!previous_observations.empty() &&
        (problem.nodes[i].position-previous_observations.front().position).norm()<.35)
      problem.preferred_first=i;
  }
  // Evaluate a sparse CP entry with the same topology search as observations.
  // Try another CP entry only when the earlier one has no connected approach.
  for (const auto &anchor:anchors) {
    if (elapsed()>cfg.edge_budget_ms) break;
    CoverageTarget entry=anchor;
    if (!selectSafeCoverageApproach(entry,false)) continue;
    TopoNode::Ptr exit=std::make_shared<TopoNode>();
    exit->center_=entry.approach_position.cast<float>();exit->is_viewpoint_=true;exit->yaw_=origin.yaw;
    vector<TopoNode::Ptr> temporary{exit};
    planner_manager_->topo_graph_->insertNodes(temporary,false);
    std::vector<EdgeSafetyCost> exit_edges(observation_end);
    bool connected=false;
    for (int i=1;i<observation_end;++i) {
      if (elapsed()>cfg.edge_budget_ms) break;
      auto candidate=candidates[problem.nodes[i].source];
      if ((candidate->center_-exit->center_).norm()<.1) {
        exit_edges[i].total_cost=0;exit_edges[i].path_length=.001;
      } else exit_edges[i]=getPathEdgeCost(candidate,Eigen::Vector3d::Zero(),candidate->yaw_,exit,exit->yaw_);
      connected=connected || exit_edges[i].path_length>0;
    }
    planner_manager_->topo_graph_->removeNodes(temporary);
    if (!connected) continue;
    Node node;node.anchor=0;node.position=entry.approach_position;node.identity=entry.stable_id;
    node.region=entry.region_id;node.yaw=origin.yaw;problem.nodes.push_back(node);problem.anchors=1;
    for (auto &row:problem.edges) row.resize(problem.nodes.size());
    problem.edges.emplace_back(problem.nodes.size());
    for (auto &row:paths) row.resize(problem.nodes.size());
    paths.emplace_back(problem.nodes.size());
    for (int i=1;i<observation_end;++i) {
      copyEdge(exit_edges[i],i,observation_end,false);
      // CP may precede deferred frontier tasks in the joint order. These
      // reverse edges describe its hypothetical suffix, never commands.
      auto reverse=problem.edges[i][observation_end];
      reverse.first=-problem.edges[i][observation_end].last;
      reverse.last=-problem.edges[i][observation_end].first;
      problem.edges[observation_end][i]=reverse;
    }
    if (coverage_intention_ids_.empty() || coverage_intention_ids_.front()!=entry.stable_id)
      coverage_intention_since_=now;
    coverage_intention_ids_={entry.stable_id};
    break;
  }
  if (!problem.anchors) coverage_intention_ids_.clear();
  // Spend the remaining topology budget across groups before alternatives;
  // avoid starving distant groups behind all pairs of the first few views.
  for (int offset=1;offset<observation_end;++offset) {
    for (int i=1;i+offset<observation_end;++i) {
      if (elapsed()>cfg.edge_budget_ms) break;
      const int j=i+offset;
      if (problem.nodes[i].group==problem.nodes[j].group) continue;
      auto a=candidates[problem.nodes[i].source],b=candidates[problem.nodes[j].source];
      const auto edge=getPathEdgeCost(a,Eigen::Vector3d::Zero(),a->yaw_,b,b->yaw_);
      copyEdge(edge,i,j,edge.known_free_length+.05>=edge.path_length);
      Edge reverse=problem.edges[i][j];reverse.first=-problem.edges[i][j].last;
      reverse.last=-problem.edges[i][j].first;problem.edges[j][i]=reverse;
      paths[j][i]=paths[i][j];std::reverse(paths[j][i].begin(),paths[j][i].end());
    }
  }

  int oldest=-1;
  for (int g=0;g<problem.groups;++g) {
    const auto &task=coverage_route_tasks_[group_ids[g]];
    double age=0.0;
    for (int i=1;i<observation_end;++i) if (problem.nodes[i].group==g)
      age=std::max(age,candidates[problem.nodes[i].source]->frontier_wait_age_);
    if (age>=2*cfg.intention_hold && task.deferred>=cfg.mandatory_after &&
        (oldest<0 || task.deferred>coverage_route_tasks_[group_ids[oldest]].deferred)) oldest=g;
  }
  // One overdue reachable task becomes mandatory, not an ever-growing reward.
  if (oldest>=0) problem.mandatory=1U<<oldest;
  auto result=solve(problem,cfg);
  if (!result.valid && problem.entry_groups) {
    // The next CP region can temporarily have no forward/reachable action.
    // Preserve coverage tasks and permit another safe entry before retrying.
    problem.entry_groups=0;result=solve(problem,cfg);
  }
  if (!result.valid && problem.anchors>0) {
    // A stale or budget-incomplete exit must not prevent connected local work.
    // Remove the hypothesis, retain all observation obligations and retry.
    problem.nodes.resize(observation_end);problem.anchors=0;
    problem.edges.resize(observation_end);
    for (auto &row:problem.edges) row.resize(observation_end);
    coverage_intention_ids_.clear();result=solve(problem,cfg);
  }
  if (!result.valid) {
    // Explicit safe singleton fallback, preserving all unexecuted groups.
    // Alternatives must never fall through into the legacy all-node ATSP.
    int best=-1;
    for (int i=1;i<observation_end;++i) {
      if (!problem.edges[0][i].executable || (oldest>=0 && problem.nodes[i].group!=oldest)) continue;
      if (best<0 || problem.edges[0][i].length<problem.edges[0][best].length) best=i;
    }
    if (best<0) for (int i=1;i<observation_end;++i)
      if (problem.edges[0][i].executable && (best<0 || problem.edges[0][i].length<problem.edges[0][best].length)) best=i;
    if (best<0) return false;
    result.prefix={best};result.route={best};result.valid=true;
    ROS_WARN_STREAM_THROTTLE(1.0,"[coverage route] safe singleton fallback; anchors=" << problem.anchors);
  }
  std::unordered_set<int> executing;
  if (!result.prefix.empty()) {
    const auto &first=problem.nodes[result.prefix.front()];
    auto &task=coverage_route_tasks_[first.identity];
    if (!first.stop) {
      const double distance=(first.position-pos).norm();
      const double fraction=frontier_manager_ptr_->observedBoundaryFraction(task.cells);
      const double timeout=std::max(3.0,cfg.intention_hold);
      coverage_route_stalled_=task.progress.observe(now,distance,fraction,timeout);
      if (coverage_route_stalled_) ROS_WARN_STREAM("[coverage route] no progress task=" << first.identity
          << " duration=" << now-task.progress.progress_time << " distance=" << distance << " evidence=" << fraction);
    }
  }
  for (int index:result.prefix) {
    prefix.push_back(problem.nodes[index].source);executing.insert(problem.nodes[index].group);
  }
  // Keep geometry for the planned suffix even beyond the current executable
  // prefix. The FSM may extend into its freshly observed part; it submits
  // only observation gates that the concrete trajectory actually contains.
  int previous=0;
  for (int index:result.route) {
    const auto &node=problem.nodes[index];
    if (node.anchor>=0) {coverage_route_exit_path_=paths[previous][index];break;}
    RouteObservation observation;observation.position=node.position;observation.yaw=node.yaw;
    observation.cluster=candidates[node.source]->frontier_cluster_id_;observation.identity=node.identity;
    observation.map_version=snapshot ? snapshot->map_version : 0;
    observation.path_from_previous=paths[previous][index];previous=index;
    coverage_route_observations_.push_back(observation);
    if (node.stop) break;
  }
  for (int g=0;g<problem.groups;++g) {
    auto &task=coverage_route_tasks_[group_ids[g]];
    if (!executing.count(g)) ++task.deferred;
    // Reset only on actual passage/evidence, not simply because it was planned.
  }
  std::ostringstream log; log << "[coverage route] map=" << (snapshot?snapshot->map_version:0)
      << " tasks=" << problem.groups << " anchors=" << problem.anchors << " prefix=" << prefix.size()
      << " complete_route=" << result.complete << " budget_exhausted=" << result.budget_exhausted
      << " deferred=" << problem.groups-executing.size() << " mandatory=" << oldest
      << " entry_groups=" << problem.entry_groups
      << " solve_ms=" << result.compute_ms << " total_ms=" << elapsed() << " order=";
  for (int index:result.route) log << (problem.nodes[index].anchor>=0?'A':'V') << problem.nodes[index].identity << ',';
  ROS_INFO_STREAM(log.str());
  return !prefix.empty();
}
}  // namespace fast_planner
