#pragma once

#include <cmath>
#include <cstdint>
#include <exception>
#include <string>

#include <rog_map/rog_map.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace general_planner::pointcloud {

/** Convert a standard XYZ or XYZI PointCloud2 without requiring intensity. */
inline bool toRogPointCloud(const sensor_msgs::PointCloud2& input,
                            rog_map::PointCloud& output,
                            std::string* error = nullptr) {
  output.clear();
  output.header.frame_id = input.header.frame_id;
  output.header.stamp = input.header.stamp.toNSec() / 1000ULL;

  bool has_intensity = false;
  for (const auto& field : input.fields) {
    if (field.name == "intensity") {
      has_intensity = true;
      break;
    }
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(input, "z");
    if (has_intensity) {
      sensor_msgs::PointCloud2ConstIterator<float> intensity(input, "intensity");
      for (; x != x.end(); ++x, ++y, ++z, ++intensity) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        rog_map::PclPoint point;
        point.x = *x;
        point.y = *y;
        point.z = *z;
        point.intensity = *intensity;
        output.points.push_back(point);
      }
    } else {
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        rog_map::PclPoint point;
        point.x = *x;
        point.y = *y;
        point.z = *z;
        point.intensity = 0.0F;
        output.points.push_back(point);
      }
    }
  } catch (const std::exception& exception) {
    output.clear();
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  }

  output.width = static_cast<std::uint32_t>(output.points.size());
  output.height = 1;
  output.is_dense = true;
  return true;
}

}  // namespace general_planner::pointcloud
