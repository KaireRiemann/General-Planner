#include "core/mapping_manager.h"
#include <cmath>

namespace person_tracker
{
void MappingRos::publishTargetPath(const std_msgs::Header & header, bool valid)
{
  nav_msgs::Path path;
  path.header = header;
  if (valid && !trackers_.empty()) {
    const auto & tracker = *trackers_.front();
    const int steps = static_cast<int>(std::floor(path_horizon_ / path_step_ + 1e-9));
    path.poses.reserve(steps + 1);
    for (int i = 0; i <= steps; ++i) {
      const double t = i * path_step_;
      const auto point = tracker.futurePosition(t);
      const double yaw = tracker.futureYaw(t);
      if (!point.allFinite() || !std::isfinite(yaw)) {
        path.poses.clear();
        break;
      }
      geometry_msgs::PoseStamped pose;
      pose.header = header;
      pose.header.stamp += ros::Duration(t);
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      path.poses.push_back(pose);
    }
  }
  target_path_pub_.publish(path);
}
}  // namespace person_tracker
