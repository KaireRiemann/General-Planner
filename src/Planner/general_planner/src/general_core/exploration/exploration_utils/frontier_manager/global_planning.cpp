/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-07-12 10:30:16
 * @LastEditTime: 2024-07-12 21:35:39
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#include <general_core/exploration/exploration_utils/frontier_manager/frontier_manager.h>

namespace {
enum class PlanningRejectReason {
  NONE,
  STATE
};

PlanningRejectReason clusterPlanningRejectReason(
    const ClusterInfo::Ptr &cluster, const FrontierParam &param) {
  (void)param;
  if (!cluster) {
    return PlanningRejectReason::STATE;
  }
  if (cluster->state_ == FrontierState::VISITED ||
      cluster->state_ == FrontierState::BLACKLISTED) {
    return PlanningRejectReason::STATE;
  }
  return PlanningRejectReason::NONE;
}

}  // namespace

class UF {
public:
  UF(int size) {
    father.resize(size);
    rank.resize(size, 0);
    for (int i = 0; i < size; ++i) {
      father[i] = i;
    }
  }

  int find(int x) {
    if (x != father[x]) {
      father[x] = find(father[x]); // Path compression
    }
    return father[x];
  }

  void connect(int x, int y) {
    int xx = find(x), yy = find(y);
    if (xx != yy) {
      if (rank[xx] < rank[yy]) {
        father[xx] = yy;
      } else if (rank[xx] > rank[yy]) {
        father[yy] = xx;
      } else {
        father[yy] = xx;
        rank[xx]++;
      }
    }
  }

private:
  vector<int> father;
  vector<int> rank;
};



void FrontierManager::generateTSPViewpoints(
    Eigen::Vector3f &center, vector<TopoNode::Ptr> &viewpoints) {
  static const std::unordered_set<int> no_preference;
  generateTSPViewpoints(center, viewpoints, no_preference);
}

void FrontierManager::generateTSPViewpoints(
    Eigen::Vector3f&center, vector<TopoNode::Ptr> &viewpoints,
    const std::unordered_set<int> &preferred_cluster_ids) {

  unordered_set<ClusterInfo::Ptr> revp_clusters_set; // (re)-generate viewpoints clusters
  struct CandidateCluster {
    ClusterInfo::Ptr cluster;
    float distance{0.0f};
  };
  vector<CandidateCluster> candidate_clusters;
  const HighSpeedViewScoreContext ctx = high_speed_view_ctx_;
  const bool use_corridor_bias =
      ctx.enabled && ctx.forward_known_free && ctx.corridor_cruise_enable;
  Eigen::Vector3f heading_dir(std::cos(ctx.curr_yaw), std::sin(ctx.curr_yaw), 0.0f);
  if (ctx.curr_vel.norm() > 0.5f) {
    heading_dir = ctx.curr_vel.normalized();
  }
  if (heading_dir.norm() < 1.0e-3f) {
    heading_dir = Eigen::Vector3f::UnitX();
  }
  const double heading_known_free =
      use_corridor_bias
          ? ctx.forward_known_free(ctx.curr_pos.cast<double>(),
                                   heading_dir.cast<double>(),
                                   ctx.known_free_max_len, ctx.min_clearance,
                                   ctx.query_step)
          : 0.0;
  const bool corridor_cruise_mode =
      use_corridor_bias && heading_known_free >= ctx.corridor_known_free_len;
  auto computeCandidateDistance = [&](const ClusterInfo::Ptr &cluster) {
    float distance = graph_->estimateRoughDistance(cluster->center_, cluster->odom_id_);
    if (corridor_cruise_mode) {
      Eigen::Vector3f to_cluster = cluster->center_ - ctx.curr_pos;
      const float cluster_dist = to_cluster.norm();
      if (cluster_dist > 1.0e-3f) {
        const float align =
            std::clamp(to_cluster.normalized().dot(heading_dir), -1.0f, 1.0f);
        distance -= static_cast<float>(
            ctx.corridor_forward_weight * std::max(0.0f, align) *
            std::min<double>(cluster_dist, ctx.known_free_max_len));
        if (align < ctx.corridor_min_alignment) {
          distance += static_cast<float>(
              ctx.corridor_lateral_penalty *
              (ctx.corridor_min_alignment - align));
        }
        if (align < 0.0f) {
          distance += static_cast<float>(ctx.corridor_lateral_penalty * (-align));
        }
      }
    }
    return distance;
  };
  for (auto &cluster : cluster_list_) {
    if (revp_clusters_set.count(cluster))
      continue;
    if (clusterPlanningRejectReason(cluster, frtp_) != PlanningRejectReason::NONE) {
      continue;
    }
    candidate_clusters.push_back({cluster, computeCandidateDistance(cluster)});
  }

  // The legacy frontend regenerated only the nearest local_tsp_size clusters.
  // Coverage guidance may identify a side-room cluster outside that Euclidean
  // shortlist, so inject only its bounded current/next-region set. The normal
  // reachability and safety gates below still decide whether it is executable.
  for (const CandidateCluster &candidate : candidate_clusters) {
    if (preferred_cluster_ids.count(candidate.cluster->id_)) {
      candidate.cluster->is_new_cluster_ = false;
      revp_clusters_set.insert(candidate.cluster);
    }
  }

  vector<int> idx;
  for (int i = 0; i < static_cast<int>(candidate_clusters.size()); i++) {
    idx.push_back(i);
  }

  sort(idx.begin(), idx.end(), [&](int a, int b) {
    return candidate_clusters[a].distance < candidate_clusters[b].distance;
  });

  int consider_range = min(vpp_.local_tsp_size_, (int)idx.size());
  // cout << "old_clusters_within_consideration num: " << consider_range << endl;
  for (int i = 0; i < consider_range; i++) {
    candidate_clusters[idx[i]].cluster->is_new_cluster_ = false;
    revp_clusters_set.insert(candidate_clusters[idx[i]].cluster);
  }
  // 附近的+新生成的
  vector<ClusterInfo::Ptr> revp_clusters_vec; // revp: regenerate viewpoint
  revp_clusters_vec.insert(revp_clusters_vec.end(), revp_clusters_set.begin(), revp_clusters_set.end());
  ros::Time t1 = ros::Time::now();
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &cluster : revp_clusters_vec) {
    initClusterViewpoints(cluster);
  }
  ros::Time t2 = ros::Time::now();
  // cout << "init cluster viewpoint cost: " << (t2 - t1).toSec() * 1000 << "ms" << endl;

  if (frtp_.view_cluster_) {
    PointVector vp_centers;
    for (auto &cls : revp_clusters_vec) {
      for (auto &vpc : cls->vp_clusters_) {
        vp_centers.emplace_back(vpc.center_.x(), vpc.center_.y(),
                                vpc.center_.z());
      }
    }
    viz_point(vp_centers, "viewpoint_centers");
  }

  removeUnreachableViewpoints(revp_clusters_vec);
  vector<ClusterInfo::Ptr> clusters_can_be_searched_;
  for (auto &cluster : revp_clusters_vec) {
    if (cluster->is_reachable_)
      clusters_can_be_searched_.push_back(cluster);
  }

  ros::Time t3 = ros::Time::now();
  // cout << "remove unreachable cluster cost: " << (t3 - t2).toSec() * 1000 << "ms" << endl;
  // cout << "revp cluster size: " << revp_clusters_vec.size() << endl;
  // cout << "reab cluster size: " << clusters_can_be_searched_.size() << endl;
  // updateHalfSpaces(clusters_can_be_searched_);
  vector<ClusterInfo::Ptr> tsp_clusters;
  mutex mtx;
  unordered_set<int> cluster2remove;
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < static_cast<int>(clusters_can_be_searched_.size()); i++) {
    auto cluster = clusters_can_be_searched_[i];
    selectBestViewpoint(cluster);
    if (!cluster->is_reachable_)
      continue;
    mtx.lock();
    tsp_clusters.push_back(cluster);
    if (cluster->state_ == FrontierState::BLACKLISTED) {
      cluster2remove.insert(cluster->id_);
    }
    mtx.unlock();
  }
  // 飞到但看不到，说明odom漂了，这篇工作不处理，直接跳过
  cluster_list_.remove_if([&](ClusterInfo::Ptr cluster) {
    bool remove = cluster2remove.count(cluster->id_);
    if (remove) {
      for (auto &cell : cluster->cells_) {
        Eigen::Vector3i idx;
        pos2idx(cell, idx);
        ByteArrayRaw bytes;
        idx2bytes(idx, bytes);
        frtd_.label_map_[bytes] = DENSE;
      }
    }
    return remove;
  });
  ros::Time t4 = ros::Time::now();
  // cout << "select best viewpoint cost: " << (t4 - t3).toSec() * 1000 << "ms" << endl;
  // 重新topK
  vector<float> distance2odom2;
  vector<int> idx2;
  for (int i = 0; i < static_cast<int>(tsp_clusters.size()); i++) {
    float distance = tsp_clusters[i]->distance_;
    distance2odom2.push_back(distance);
    idx2.push_back(i);
  }
  if (idx2.empty()) {
    viewpoints.clear();
    ROS_INFO("vp cluster cost: %fms  ,remove unreachable cost: %fms, select vp cost: %fms",
             (t2 - t1).toSec() * 1000, (t3 - t2).toSec() * 1000,
             (t4 - t3).toSec() * 1000);
    return;
  }
  sort(idx2.begin(), idx2.end(), [&](int a, int b) { return distance2odom2[a] < distance2odom2[b]; });
  viewpoints.clear();
  for (int i = 0; i < (int)idx2.size(); i++) {
    // 剔除异常值
    // if (i > (int)(idx2.size() / 2.0) && distance2odom2[idx2[i]] > mean_distance * 5.0)
    //   break;
    const auto &cluster = tsp_clusters[idx2[i]];
    // A frontier cluster is a generalized-TSP choice: its alternative
    // viewpoints are mutually exclusive.  Treating every alternative as a
    // mandatory ATSP node duplicated frontiers and made the first goal jump
    // between two poses of the same room.  Keep the remaining alternatives in
    // ClusterInfo for local fallback, and expose only the best representative
    // to global routing.
    TopoNode::Ptr vp_node = make_shared<TopoNode>();
    vp_node->is_viewpoint_ = true;
    vp_node->frontier_cluster_id_ = cluster->id_;
    if (cluster->candidate_vps_.empty()) {
      vp_node->center_ = cluster->best_vp_;
      vp_node->yaw_ = cluster->best_vp_yaw_;
    } else {
      vp_node->center_ = cluster->candidate_vps_.front();
      vp_node->yaw_ = cluster->candidate_yaws_.empty()
                          ? cluster->best_vp_yaw_
                          : cluster->candidate_yaws_.front();
    }
    const ros::Time now = ros::Time::now();
    vp_node->frontier_information_gain_ =
        std::max(cluster->last_visible_gain_, cluster->stable_visible_gain_);
    vp_node->frontier_wait_age_ =
        cluster->first_reachable_time_.isZero()
            ? 0.0
            : std::max(0.0, (now - cluster->first_reachable_time_).toSec());
    vp_node->frontier_pass_debt_ = std::max(0.0, cluster->pass_debt_);
    vp_node->frontier_pass_count_ = std::max(0, cluster->pass_count_);
    viewpoints.push_back(vp_node);
    if (static_cast<int>(viewpoints.size()) >= vpp_.global_recluster_size_) {
      break;
    }
  }
  ROS_INFO("vp cluster cost: %fms  ,remove unreachable cost: %fms, select vp cost: %fms",
           (t2 - t1).toSec() * 1000, (t3 - t2).toSec() * 1000,
           (t4 - t3).toSec() * 1000);
}
