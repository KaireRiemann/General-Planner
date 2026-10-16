// Imported from EPICON; namespace and runtime integration adapted for General Planner.
/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-03-01 15:10:53
 * @LastEditTime: 2024-03-01 17:18:07
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <epicon_native/pointcloud_topo/parallel_bubble_astar.h>

namespace epicon_native {

namespace {
// Cover the whole segment with box intervals. Point sampling can miss small
// gaps between floors or cut across the outside corner of a non-convex union.
bool segmentInTaskBounds(const LIOInterface::Ptr &map,
                         const Eigen::Vector3f &a, const Eigen::Vector3f &b) {
  auto interval = [&](const Eigen::Vector3f &lo, const Eigen::Vector3f &hi,
                      double &enter, double &leave) {
    enter=0.0; leave=1.0;
    for(int axis=0;axis<3;++axis) {
      const double delta=double(b[axis])-a[axis];
      if(std::abs(delta)<1e-12) {
        if(a[axis]<lo[axis] || a[axis]>hi[axis]) return false;
      } else {
        double t0=(double(lo[axis])-a[axis])/delta;
        double t1=(double(hi[axis])-a[axis])/delta;
        if(t0>t1) std::swap(t0,t1);
        enter=std::max(enter,t0); leave=std::min(leave,t1);
        if(enter>leave) return false;
      }
    }
    return true;
  };
  for(int i=0;i<map->lp_->dead_area_num_;++i) {
    double enter,leave;
    if(interval(map->lp_->dead_area_min_boundary_vec_[i],
                map->lp_->dead_area_max_boundary_vec_[i],enter,leave)) return false;
  }
  for(int i=0;i<map->lp_->box_num_;++i) {
    const auto &lo=map->lp_->global_box_min_boundary_vec_[i];
    const auto &hi=map->lp_->global_box_max_boundary_vec_[i];
    if((a.array()>=lo.array()).all() && (a.array()<=hi.array()).all() &&
       (b.array()>=lo.array()).all() && (b.array()<=hi.array()).all()) return true;
  }
  std::vector<std::pair<double,double>> intervals;
  for(int i=0;i<map->lp_->box_num_;++i) {
    double enter,leave;
    if(interval(map->lp_->global_box_min_boundary_vec_[i],
                map->lp_->global_box_max_boundary_vec_[i],enter,leave))
      intervals.emplace_back(enter,leave);
  }
  std::sort(intervals.begin(),intervals.end());
  double covered=0.0;
  for(const auto &range:intervals) {
    if(range.first>covered) return false;
    covered=std::max(covered,range.second);
    if(covered>=1.0) return true;
  }
  return false;
}
// Non-overlapping endpoint spheres do not prove that a segment is blocked.
// March with steps bounded by the measured clearance surplus: distance to the
// obstacle set is 1-Lipschitz, so every covered interval remains certified.
bool segmentHasClearance(const LIOInterface::Ptr &map, const Eigen::Vector3f &a,
                         const Eigen::Vector3f &b, double clearance) {
  if(!segmentInTaskBounds(map,a,b)) return false;
  const double length=(b-a).norm();
  if(length<1e-6) return map->getDisToOcc(a)>clearance+1e-4;
  const Eigen::Vector3f direction=(b-a)/length;
  double covered=0;
  for(int n=0;n<4096;++n) {
    const Eigen::Vector3f point=a+covered*direction;
    const double surplus=map->getDisToOcc(point)-clearance;
    if(!std::isfinite(surplus) || surplus<=1e-4) return false;
    if(length-covered<surplus-1e-4) return true;
    covered+=std::min(.2,.8*(surplus-1e-4));
  }
  return false;
}
}

void ParallelBubbleAstar::init(ros::NodeHandle &nh, const LIOInterface::Ptr &lidar_map) {
  nh.param("bubble_astar/resolution_astar", resolution_, 0.1);
  nh.param("bubble_astar/lambda_heu", lambda_heu_, 1.0);
  nh.param("bubble_astar/allocate_num", allocate_num_, -1);
  nh.param("bubble_astar/safe_distance", safe_distance_, -1.0);
  nh.param("bubble_astar/debug", debug_, false);
  tie_breaker_ = 1.0 + 1.0 / 1000;
  this->lidar_map_interface_ = lidar_map;
  origin_ = lidar_map->lp_->global_map_min_boundary_;
  this->inv_resolution_ = 1.0 / resolution_;
  // frontier_manager_ = frontier_manager_;
}

bool ParallelBubbleAstar::isNodeSafe(Node::Ptr node, const Eigen::Vector3f &bbox_min,
    const Eigen::Vector3f &bbox_max, std::unordered_map<Eigen::Vector3i,double,v3i_hash> &radii,
    bool grid_center) {
  if(!node->position.allFinite() || !lidar_map_interface_->IsInMap(node->position) ||
     !lidar_map_interface_->IsInBox(node->position) ||
     (node->position.array()<bbox_min.array()).any() ||
     (node->position.array()>bbox_max.array()).any()) return false;
  // Cache exact grid-center distances only, within this search/map revision.
  // Start/goal can share a cell but have very different obstacle distances.
  Eigen::Vector3i idx; posToIndex(node->position,idx);
  auto cached=radii.find(idx);
  if(grid_center && cached!=radii.end()) node->radius=cached->second;
  else {
    node->radius=lidar_map_interface_->getDisToOcc(node->position)-safe_distance_;
    if(grid_center) radii.emplace(idx,node->radius);
  }
  return std::isfinite(node->radius) && node->radius>1e-3;
}

int ParallelBubbleAstar::search(const Eigen::Vector3f &start, const Eigen::Vector3f &goal, vector<Eigen::Vector3f> &path, double timeout,
                                bool best_result, bool only_raycast, const Eigen::Vector3f &bbox_min, const Eigen::Vector3f &bbox_max) {
  path.clear();
  const auto deadline=ros::WallTime::now()+ros::WallDuration(std::max(0.0,timeout));
  std::unordered_map<Eigen::Vector3i,double,v3i_hash> radii;
  radii.reserve(1000);
  auto curr_node=std::make_shared<Node>(); auto end_node=std::make_shared<Node>();
  curr_node->position=start; end_node->position=goal;
  if(!isNodeSafe(curr_node,bbox_min,bbox_max,radii)) return START_FAIL;
  if(!isNodeSafe(end_node,bbox_min,bbox_max,radii)) return END_FAIL;
  if(segmentHasClearance(lidar_map_interface_,start,goal,safe_distance_)) {
    path={start,goal}; return REACH_END;
  }
  const double goal_r=std::min(1.5,end_node->radius-1e-3);
  Eigen::Vector3f curr_pos=start;
  const double length=(goal-start).norm();
  const Eigen::Vector3f dir=length>1e-6 ? ((goal-start)/length).eval() : Eigen::Vector3f::Zero();
  vector<Eigen::Vector3f> direct{start};
  double radius=curr_node->radius;
  while(radius>1.1e-2) {
    if(ros::WallTime::now()>=deadline) return TIME_OUT;
    const double step=std::min(1.5,radius-1e-3);
    if(step+goal_r>(goal-curr_pos).norm()) {
      direct.push_back((curr_pos+goal)/2.0); direct.push_back(goal);
      if(collisionCheck_shortenPath(direct)) { path.swap(direct); return REACH_END; }
      break;
    }
    curr_pos+=step*dir;
    if(!lidar_map_interface_->IsInBox(curr_pos) ||
       (curr_pos.array()<bbox_min.array()).any() || (curr_pos.array()>bbox_max.array()).any()) break;
    direct.push_back(curr_pos);
    radius=lidar_map_interface_->getDisToOcc(curr_pos)-safe_distance_;
  }
  if(only_raycast) return NO_PATH;

  // 启发式函数，用于在A*算法中估算从当前节点x1到目标节点x2的预估代价
  auto getHeu = [&](const Eigen::Vector3f &x1, const Eigen::Vector3f &x2) -> double {
    if (best_result) {
      return tie_breaker_ * (x1 - x2).norm() * 0.9;
    }
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));
    return tie_breaker_ * (dx + dy + dz);
  };

  // 到达目标判断函数
  auto arriveGoal = [&](const Eigen::Vector3f &curr_pos, double curr_radius) -> bool {
    double curr_r = curr_radius;
    double dis2goal = (curr_pos - goal).norm();
    return (curr_r + goal_r > dis2goal);
  };

  // 初始化开放集和封闭集
  std::priority_queue<Node::Ptr, std::vector<Node::Ptr>, NodeCompre> open_set;
  std::unordered_map<Eigen::Vector3i, Node::Ptr, v3i_hash> open_set_map;
  std::unordered_set<Eigen::Vector3i, v3i_hash> close_set;
  close_set.reserve(1000);
  open_set_map.reserve(1000);
  curr_node->g_score = 0.0;
  if (!best_result)
    curr_node->f_score = lambda_heu_ * getHeu(start, goal);
  else
    curr_node->f_score = getHeu(start, goal);

  open_set.push(curr_node);
  Eigen::Vector3i curr_idx;
  posToIndex(curr_node->position, curr_idx);
  open_set_map.insert(make_pair(curr_idx, curr_node));

  auto backtrack = [&](Node::Ptr curr_node) {
    while (curr_node->parent != nullptr) {
      path.push_back(curr_node->position);
      curr_node = curr_node->parent;
    }
    path.push_back(curr_node->position);
    std::reverse(path.begin(), path.end());
    path.push_back(goal);
  };


  // A*主循环
  while (!open_set.empty()) {
    auto curr_node = open_set.top(); open_set.pop();
    posToIndex(curr_node->position,curr_idx);
    auto current_entry=open_set_map.find(curr_idx);
    if(current_entry==open_set_map.end() || current_entry->second!=curr_node) continue;
    const double curr_radius=curr_node->radius;
    // 检查是否到达目标
    if (arriveGoal(curr_node->position, curr_radius)) {
      backtrack(curr_node); // 回溯路径
      if (collisionCheck_shortenPath(path)) return REACH_END;
      // Sphere overlap can span the outside of a non-convex task box.
      // Keep searching for a route inside the union instead of aborting.
      path.clear();
    }

    // 处理超时
    if (ros::WallTime::now() >= deadline) {
      // ROS_ERROR("ParallelBubbleAstar: TIME_OUT");
      return ParallelBubbleAstar::TIME_OUT;
    }

    posToIndex(curr_node->position, curr_idx);
    open_set_map.erase(curr_idx);
    close_set.insert(curr_idx);
    // 扩展邻居节点
    vector<Eigen::Vector3f> step_lis{
      Eigen::Vector3f(resolution_, 0, 0),  Eigen::Vector3f(-resolution_, 0, 0), Eigen::Vector3f(0, resolution_, 0),
      Eigen::Vector3f(0, -resolution_, 0), Eigen::Vector3f(0, 0, resolution_),  Eigen::Vector3f(0, 0, -resolution_),
    };
    Eigen::Vector3f nbr_pos;

    // 处理每个邻居节点
    for (auto &step : step_lis) 
    // for(int x=-1;x<=1;x++)
    // for(int y=-1;y<=1;y++)
    // for(int z=-1;z<=1;z++)
    {
      // if(x==0&&y==0&&z==0)
      //   continue;
      // Eigen::Vector3f step = Eigen::Vector3f(x*resolution_, y*resolution_, z*resolution_);
      nbr_pos = curr_node->position + step;
      Eigen::Vector3i nbr_idx;
      posToIndex(nbr_pos, nbr_idx);
      IndexToPos(nbr_pos, nbr_idx);

      if (close_set.find(nbr_idx) != close_set.end())
        continue;
      auto nei_node = std::make_shared<Node>();
      nei_node->position = nbr_pos;

      if (!isNodeSafe(nei_node, bbox_min, bbox_max, radii, true))
        continue;
      // Certify each edge with the same overlapping free spheres used by
      // collisionCheck_shortenPath. Safe endpoints alone can cross a corner;
      // selecting that parent used to reject the whole route at the end.
      const double next_radius=nei_node->radius;
      if (std::min(2.0,curr_radius)+std::min(2.0,next_radius)+1e-3 <
          (nbr_pos-curr_node->position).norm() &&
          !segmentHasClearance(lidar_map_interface_,curr_node->position,nbr_pos,safe_distance_)) continue;
      if(!segmentInTaskBounds(lidar_map_interface_,curr_node->position,nbr_pos)) continue;
      const double tmp_g_score=(nbr_pos-curr_node->position).norm()+curr_node->g_score;
      auto node_iter=open_set_map.find(nbr_idx);
      if(node_iter!=open_set_map.end() && tmp_g_score>=node_iter->second->g_score) continue;
      nei_node->parent=curr_node;
      nei_node->g_score=tmp_g_score;
      nei_node->f_score=(best_result ? 1.0 : lambda_heu_)*getHeu(nbr_pos,goal)+tmp_g_score;
      // Immutable heap entries: a better parent replaces the map entry and
      // old heap entries are skipped on pop. The old relaxation did neither.
      open_set_map[nbr_idx]=nei_node;
      open_set.push(nei_node);
    }
  }
  // ROS_INFO("all space searched");
  return ParallelBubbleAstar::NO_PATH;
}

void ParallelBubbleAstar::reset() {
  unordered_set<Eigen::Vector3i, v3i_hash>().swap(safe_node);
  unordered_set<Eigen::Vector3i, v3i_hash>().swap(dangerous_node);
}

void ParallelBubbleAstar::posToIndex(const Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

void ParallelBubbleAstar::IndexToPos(Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
  pt = ((idx.cast<float>() + Eigen::Vector3f(0.5, 0.5, 0.5)) * resolution_ + origin_);
}

bool ParallelBubbleAstar::collisionCheck_shortenPath(vector<Eigen::Vector3f> &path) {
  // if (path.size() < 2)
  //   return false;
  // vector<Eigen::Vector3f> path_shorten;
  // std::vector<double> raduis_lis;
  // for (int i = 0; i < path.size(); i++) {
  //   double dis = lidar_map_interface_->getDisToOcc(path[i]) - safe_distance_;
  //   raduis_lis.push_back(dis);
  //   if (raduis_lis.back() < 1e-3) {
  //     return false;
  //   }
  // }
  // vector<int> path_shorten_idx;
  // path_shorten_idx.push_back(0);
  // for (int i = 1; i < path.size() - 1; i++) {
  //   Eigen::Vector3f back_node = path[path_shorten_idx.back()];
  //   Eigen::Vector3f cur = path[i];
  //   Eigen::Vector3f next = path[i + 1];
  //   bool cur_connect = (cur - back_node).norm() < raduis_lis[i] + raduis_lis[path_shorten_idx.back()];
  //   bool next_connect = ((next - back_node).norm() < raduis_lis[i + 1] + raduis_lis[path_shorten_idx.back()]);
  //   if (cur_connect && next_connect)
  //     continue;
  //   else if (cur_connect && !next_connect) {
  //     path_shorten_idx.push_back(i);
  //   } else if (!cur_connect) {
  //     return false;
  //   }
  // }
  // path_shorten_idx.push_back(path.size() - 1);

  // for (int i = 0; i < path_shorten_idx.size() ; i++) {
  //   path_shorten.push_back(path[path_shorten_idx[i]]);
  // }
  // path.swap(path_shorten);
  // return true;
  if (path.size() < 2)
    return false;
  for(size_t i=0;i<path.size();++i) {
    if(!path[i].allFinite() || !lidar_map_interface_->IsInMap(path[i]) ||
       !lidar_map_interface_->IsInBox(path[i])) return false;
    if(i && !segmentInTaskBounds(lidar_map_interface_,path[i-1],path[i])) return false;
  }
  std::vector<Eigen::Vector3f> path_shorten;
  std::vector<double> raduis_lis;
  for (int i = 0; i < path.size(); i++) {
    double dis = min(2.0, lidar_map_interface_->getDisToOcc(path[i]) - safe_distance_);
    raduis_lis.push_back(dis);
    if (raduis_lis.back() < 1e-3) {
      return false;
    }
  }

  std::deque<int> indices;
  indices.push_front(path.size() - 1); // goal
  for (int i = path.size() - 1; i > 0; i--) {
    double i_raduis = raduis_lis[i];
    bool connected = false;
    for (int j = 0; j < i; j++) {
      double j_raduis = raduis_lis[j];
      double distance = (path[i] - path[j]).norm();
      if (i_raduis + j_raduis + 1e-3 > distance ||
          ((j==i-1 || distance<=.5) &&
           segmentHasClearance(lidar_map_interface_,path[j],path[i],safe_distance_))) {
        if(!segmentInTaskBounds(lidar_map_interface_,path[j],path[i])) continue;
        indices.push_front(j);
        connected = true;
        i = j + 1;
        break;
      }
    }
    if (!connected) {
      return false;
    }
  }
  for (auto &idx : indices) {
    path_shorten.emplace_back(path[idx]);
  }
  path.swap(path_shorten);
  return true;
}

void ParallelBubbleAstar::calculatePathCost(const vector<Eigen::Vector3f> &path, double &cost) {
  cost = 0;
  for (int i = 0; i < path.size() - 1; i++) {
    cost += (path[i + 1] - path[i]).norm();
  }
}

} // namespace epicon_native
