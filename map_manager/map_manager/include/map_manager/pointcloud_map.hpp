#pragma once

#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace general_planner
{

class PointCloudMap
{
public:
    using Ptr = std::shared_ptr<PointCloudMap>;
    using PointType = pcl::PointXYZ;
    using PointVector = pcl::PointCloud<PointType>::VectorType;

    virtual ~PointCloudMap() = default;

    virtual double getDisToOcc(const Eigen::Vector3f &pt) const
    {
        (void)pt;
        // TODO(pointcloud-map): hook this to the real nearest-obstacle query.
        return 0.0;
    }

    virtual void KNN(const PointType &pt,
                     const int k,
                     PointVector &pts,
                     std::vector<float> &sqr_distances) const
    {
        (void)pt;
        (void)k;
        // TODO(pointcloud-map): replace with kd-tree/ikd-tree nearest-neighbor search.
        pts.clear();
        sqr_distances.clear();
    }

    virtual void KNN(const Eigen::Vector3f &pt,
                     const int k,
                     PointVector &pts,
                     std::vector<float> &sqr_distances) const
    {
        PointType query;
        query.x = pt.x();
        query.y = pt.y();
        query.z = pt.z();
        KNN(query, k, pts, sqr_distances);
    }

    virtual void boxSearchPointCloud(const Eigen::Vector3f &box_min,
                                     const Eigen::Vector3f &box_max,
                                     PointVector &pts) const
    {
        (void)box_min;
        (void)box_max;
        // TODO(pointcloud-map): replace with kd-tree/ikd-tree range search.
        pts.clear();
    }

    virtual bool isInBox(const Eigen::Vector3f &pt) const
    {
        (void)pt;
        // TODO(pointcloud-map): apply exploration workspace/dead-area policy.
        return false;
    }

    virtual bool isInMap(const Eigen::Vector3f &pt) const
    {
        (void)pt;
        // TODO(pointcloud-map): apply global map boundary policy.
        return false;
    }
};

} // namespace general_planner
