#pragma once

#include <cmath>

#include <rog_map/rog_map_core/common_lib.hpp>

namespace rog_map::raycast_geometry {

/**
 * Clip a sensor-to-endpoint segment at a horizontal plane.
 *
 * The result is a segment interpolation, not a displacement along a unit
 * vector. Returning false means that the plane is not on the finite segment.
 */
inline bool clipSegmentToHeightPlane(Vec3f& endpoint,
                                     const Vec3f& sensor_origin,
                                     const double plane_height) {
    const double dz = endpoint.z() - sensor_origin.z();
    if (!std::isfinite(dz) || std::abs(dz) < 1.0e-9) {
        return false;
    }
    const double ratio = (plane_height - sensor_origin.z()) / dz;
    if (!std::isfinite(ratio) || ratio < 0.0 || ratio > 1.0) {
        return false;
    }
    endpoint = sensor_origin + (endpoint - sensor_origin) * ratio;
    return endpoint.allFinite();
}

}  // namespace rog_map::raycast_geometry
