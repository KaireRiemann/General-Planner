#include <map_manager/temporal_static_filter.hpp>

#include <cstdlib>
#include <initializer_list>
#include <iostream>

namespace {

rog_map::PclPoint makePoint(const float x, const float y, const float z) {
  rog_map::PclPoint point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 0.0F;
  return point;
}

rog_map::PointCloud makeCloud(const std::initializer_list<rog_map::PclPoint>& points) {
  rog_map::PointCloud cloud;
  cloud.points.insert(cloud.points.end(), points.begin(), points.end());
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

void expect(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "temporal_static_filter_self_test: " << message << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

}  // namespace

int main() {
  general_planner::TemporalStaticFilter filter;
  general_planner::TemporalStaticFilter::Config config;
  config.enabled = true;
  config.voxel_size = 0.30;
  config.min_observations = 4;
  config.min_observation_span = 0.30;
  config.min_observer_baseline = 0.25;
  config.max_observation_gap = 0.60;
  config.max_voxels = 32;
  filter.configure(config);

  const auto static_cloud = makeCloud({makePoint(1.05F, 2.02F, 0.61F)});
  const rog_map::Vec3f observer_0(0.0, 0.0, 0.0);
  const rog_map::Vec3f observer_1(0.08, 0.0, 0.0);
  const rog_map::Vec3f observer_2(0.16, 0.0, 0.0);
  const rog_map::Vec3f observer_3(0.22, 0.0, 0.0);
  const rog_map::Vec3f observer_4(0.30, 0.0, 0.0);
  expect(filter.filter(static_cloud, 0.00, &observer_0).empty(),
         "first static observation must not enter the map");
  expect(filter.filter(static_cloud, 0.10, &observer_1).empty(),
         "second static observation must not enter the map");
  expect(filter.filter(static_cloud, 0.20, &observer_2).empty(),
         "third static observation must not enter the map");
  expect(filter.filter(static_cloud, 0.31, &observer_3).empty(),
         "insufficient viewpoint baseline must keep a point out of the map");
  expect(filter.filter(static_cloud, 0.41, &observer_4).size() == 1,
         "persistent multi-view point must enter after the configured gates");

  filter.reset();
  expect(filter.filter(static_cloud, 0.50, &observer_0).empty(),
         "hovering baseline test frame one");
  expect(filter.filter(static_cloud, 0.60, &observer_0).empty(),
         "hovering baseline test frame two");
  expect(filter.filter(static_cloud, 0.70, &observer_0).empty(),
         "hovering baseline test frame three");
  expect(filter.filter(static_cloud, 0.81, &observer_0).empty(),
         "a fixed observer must not satisfy a multi-view requirement");

  filter.reset();
  expect(filter.filter(makeCloud({makePoint(0.0F, 0.0F, 1.0F)}), 1.00,
                       &observer_0).empty(),
         "moving point frame one must not enter the map");
  expect(filter.filter(makeCloud({makePoint(0.35F, 0.0F, 1.0F)}), 1.10,
                       &observer_1).empty(),
         "moving point frame two must not enter the map");
  expect(filter.filter(makeCloud({makePoint(0.70F, 0.0F, 1.0F)}), 1.20,
                       &observer_2).empty(),
         "moving point frame three must not enter the map");
  expect(filter.filter(makeCloud({makePoint(1.05F, 0.0F, 1.0F)}), 1.31,
                       &observer_4).empty(),
         "moving point frame four must not enter the map");

  filter.reset();
  expect(filter.filter(static_cloud, 2.00, &observer_0).empty(),
         "gap test frame one");
  expect(filter.filter(static_cloud, 2.10, &observer_1).empty(),
         "gap test frame two");
  expect(filter.filter(static_cloud, 3.00, &observer_2).empty(),
         "a gap longer than the threshold must restart promotion");
  expect(filter.filter(static_cloud, 3.10, &observer_3).empty(),
         "gap test restarted frame two");
  expect(filter.filter(static_cloud, 3.20, &observer_4).empty(),
         "gap test restarted frame three");
  const rog_map::Vec3f observer_5(0.55, 0.0, 0.0);
  expect(filter.filter(static_cloud, 3.31, &observer_5).size() == 1,
         "point must be promotable after a restarted continuous sequence");

  config.enabled = false;
  filter.configure(config);
  expect(filter.filter(static_cloud, 4.00, &observer_0).size() == 1,
         "disabled temporal filter must preserve legacy immediate fusion");

  return EXIT_SUCCESS;
}
