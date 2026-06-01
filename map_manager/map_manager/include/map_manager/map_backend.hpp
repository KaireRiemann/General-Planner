#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace general_planner
{

enum class MapBackend
{
    ROG,
    POINT_CLOUD,
    EPIC_LIO,
    LOCAL_EDT,
    HYBRID
};

inline std::string normalizeMapBackendName(std::string name)
{
    std::transform(name.begin(), name.end(), name.begin(), [](const unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    std::replace(name.begin(), name.end(), '-', '_');
    return name;
}

inline MapBackend mapBackendFromString(const std::string &name)
{
    const std::string normalized = normalizeMapBackendName(name);
    if (normalized == "point_cloud" || normalized == "pointcloud" ||
        normalized == "pc" || normalized == "pcl") {
        return MapBackend::POINT_CLOUD;
    }
    if (normalized == "epic_lio" || normalized == "epic_lio_map" ||
        normalized == "lio" || normalized == "lio_map" ||
        normalized == "epic") {
        return MapBackend::EPIC_LIO;
    }
    if (normalized == "hybrid") {
        return MapBackend::HYBRID;
    }
    if (normalized == "local_edt" || normalized == "edt" || normalized == "esdf") {
        return MapBackend::LOCAL_EDT;
    }
    return MapBackend::ROG;
}

inline std::string mapBackendToString(const MapBackend backend)
{
    switch (backend) {
        case MapBackend::POINT_CLOUD:
            return "point_cloud";
        case MapBackend::EPIC_LIO:
            return "epic_lio";
        case MapBackend::HYBRID:
            return "hybrid";
        case MapBackend::LOCAL_EDT:
            return "local_edt";
        case MapBackend::ROG:
        default:
            return "rog";
    }
}

} // namespace general_planner
