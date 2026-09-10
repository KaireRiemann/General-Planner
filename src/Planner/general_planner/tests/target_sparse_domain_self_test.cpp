#include <general_core/exploration/highspeed/target_topology_guidance.h>
#include <general_core/exploration/exploration_utils/frontier_manager/frontier_manager.h>
#include <general_core/exploration/highspeed/planner_manager.h>
#include <cassert>
#include <iostream>
#include <limits>

int main(int argc, char **argv) {
  using Eigen::Vector3f;
  ros::init(argc, argv, "target_workspace_self_test", ros::init_options::AnonymousName);
  ros::Time::init();
  auto map = std::make_shared<fast_planner::LIOInterface>();
  map->lp_.reset(new fast_planner::LIOInterfaceParam{});
  map->ld_.reset(new fast_planner::LIOInterfaceData{});
  map->setSingleExplorationBox(Vector3f(-20,-20,-.2), Vector3f(20,20.2,3.8));
  map->lp_->box_num_ = 2;
  map->lp_->global_box_min_boundary_vec_.push_back(Vector3f(-9.5,-9.5,2.8));
  map->lp_->global_box_max_boundary_vec_.push_back(Vector3f(9.5,9.5,6.4));
  map->lp_->global_box_max_boundary_.z() = 6.4;
  map->lp_->global_map_max_boundary_.z() = 6.4;
  map->lp_->max_ray_length_ = 20;
  map->lp_->dead_area_num_ = 1;
  map->lp_->dead_area_min_boundary_vec_ = {Vector3f(5,5,0)};
  map->lp_->dead_area_max_boundary_vec_ = {Vector3f(6,6,3)};
  const Vector3f start(21.5,0,1.5), remote(10000,0,1.5);
  assert(!map->IsInBox(start));
  map->setTargetNavigation(true);
  assert(map->IsInBox(start) && map->IsInBox(remote));
  assert(map->IsInMap(remote));
  assert(!map->IsInBox(Vector3f(5.5,5.5,1)));
  assert(!map->IsInBox(Vector3f(std::numeric_limits<float>::quiet_NaN(),0,0)));


  // Task-domain permission must not turn missing sensor evidence into free air.
  fast_planner::FastPlannerManager planner;
  planner.lidar_map_interface_ = map;
  assert(planner.querySafetyState(remote.cast<double>()) ==
         fast_planner::MapVoxelState::UNKNOWN);
  assert(planner.querySafetyState(Eigen::Vector3d(5.5,5.5,1)) ==
         fast_planner::MapVoxelState::OUT_OF_MAP);

  // Signed key identity survives negative cells and crossings of old extents.
  std::unordered_map<ByteArrayRaw,int,ByteArrayRawHasher> cells;
  for (int i=-10000; i<=10000; ++i) {
    ByteArrayRaw k; k.index = {{i, -i, i % 7}};
    cells.emplace(k, i);
  }
  assert(cells.size() == 20001);
  for (const auto &entry : cells) assert(entry.first.index[0] == entry.second);

  TopoGraph graph;
  graph.lidar_map_interface_ = map;
  graph.min_bd = Vector3f(-20,-20,-.2);
  graph.init_region_size_x_ = graph.init_region_size_y_ = 5;
  graph.init_region_size_z_ = 2;
  Eigen::Vector3i negative;
  graph.getIndex(Vector3f(-20.1,-20.1,-.3), negative);
  assert(negative == Eigen::Vector3i(-1,-1,-1));
  RegionNode::Ptr historical;
  Eigen::Vector3i first;
  for (int i=0; i<30; ++i) {
    map->ld_->lidar_pose_ = Vector3f(21.5 + i*25,0,1.5);
    map->ld_->lidar_cloud_.clear();
    map->ld_->lidar_cloud_.push_back(pcl::PointXYZ(25+i*25,0,1.5));
    graph.getRegionsToUpdate();
    assert(graph.hasRegionForPoint(map->ld_->lidar_pose_));
    if (i == 0) {
      graph.getIndex(map->ld_->lidar_pose_, first);
      historical = graph.getRegionNode(first);
    }
    assert(graph.getRegionNode(first) == historical);
  }
  // Coverage boxes remain unchanged after navigation and a mission switch.
  map->setTargetNavigation(false);
  assert(!map->IsInBox(start) && map->IsInBox(Vector3f(0,0,1)));
  assert(map->lp_->global_box_max_boundary_.x() == 20);
  // Coverage retains its box union and explicit exclusion semantics after
  // navigation has populated hundreds of metres of additional regions.
  for (int x = -25; x <= 25; ++x) {
    for (int y = -25; y <= 25; ++y) {
      for (int zi = -2; zi <= 28; ++zi) {
        const float z = zi * 0.25f;
        const bool in_box = (x >= -20 && x <= 20 && y >= -20 && y <= 20 &&
                             z >= -0.2f && z <= 3.8f) ||
                            (x >= -9.5 && x <= 9.5 && y >= -9.5 && y <= 9.5 &&
                             z >= 2.8f && z <= 6.4f);
        const bool excluded = x >= 5 && x <= 6 && y >= 5 && y <= 6 &&
                              z >= 0 && z <= 3;
        assert(map->IsInBox(Vector3f(x,y,z)) == (in_box && !excluded));
      }
    }
  }
  // Preserve the legacy LIO-only safety fallback in coverage, while target
  // requires current free-space evidence (tested above).
  assert(planner.querySafetyState(Eigen::Vector3d(0,0,1)) ==
         fast_planner::MapVoxelState::KNOWN_FREE);
  assert(planner.querySafetyState(remote.cast<double>()) ==
         fast_planner::MapVoxelState::OUT_OF_MAP);
  const auto regions_before = graph.reg_map_idx2ptr_.size();
  map->ld_->lidar_pose_ = Vector3f(-500,0,1.5);
  map->ld_->lidar_cloud_.clear();
  graph.getRegionsToUpdate();
  assert(!graph.hasRegionForPoint(map->ld_->lidar_pose_));
  assert(graph.reg_map_idx2ptr_.size() == regions_before);
  map->setTargetNavigation(true);
  graph.getRegionsToUpdate();
  assert(graph.hasRegionForPoint(map->ld_->lidar_pose_));
  assert(graph.getRegionNode(first) == historical);
  // Merely querying an unobserved final target does not allocate a region.
  const auto observed_count = graph.reg_map_idx2ptr_.size();
  assert(!graph.hasRegionForPoint(remote));
  assert(graph.reg_map_idx2ptr_.size() == observed_count);

  fast_planner::TargetTopologyGuidanceConfig config;
  config.anchor_candidate_count = 2;
  config.minimum_progress = .5;
  std::vector<fast_planner::TargetTopologyAnchorCandidate> anchors = {
      {Vector3f(10,0,1), 0}, {Vector3f(20,0,1), 0}, {Vector3f(-5,0,1), 0}};
  auto ranked = fast_planner::rankTargetTopologyAnchors(
      Eigen::Vector3d(0,0,1), Eigen::Vector3d(100,0,1), anchors, config);
  assert(ranked.size() == 2 && ranked.back().position.x() < 0);
  // Target history must not change coverage completion/visited semantics.
  FrontierManager frontiers;
  auto coverage_cluster = std::make_shared<ClusterInfo>();
  coverage_cluster->id_ = 10;
  coverage_cluster->state_ = FrontierState::VISITED;
  coverage_cluster->is_dormant_ = true;
  frontiers.cluster_list_.push_back(coverage_cluster);
  frontiers.setTaskDomain(true);
  assert(frontiers.cluster_list_.empty());
  auto target_cluster = std::make_shared<ClusterInfo>();
  target_cluster->id_ = 11;
  target_cluster->state_ = FrontierState::ACTIVE;
  target_cluster->is_dormant_ = false;
  frontiers.cluster_list_.push_back(target_cluster);
  frontiers.setTaskDomain(false);
  assert(frontiers.cluster_list_.size() == 1);
  assert(frontiers.cluster_list_.front() == coverage_cluster);
  assert(coverage_cluster->state_ == FrontierState::VISITED);
  assert(frontiers.activeClusterCount() == 0);
  frontiers.setTaskDomain(true);
  assert(frontiers.cluster_list_.front() == target_cluster);
  assert(frontiers.activeClusterCount() == 1);
  std::cout << "target sparse domain / mode switch / historical identity / detour: PASS\n";
}
