#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rog_map/rog_map.h>

#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>

#include <Eigen/Geometry>

#include <memory>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "utils/header/eigen_alias.hpp"
#include "ros_interface/ros1/ros1_interface.hpp"
#include "traj_opt/flatness/se3_flatness_map.hpp"
#include "traj_opt/se3_aggressive_traj_opt.hpp"

namespace
{
template <typename T>
T getParamOrDefault(ros::NodeHandle &node,
                    const std::string &name,
                    const T &default_value)
{
    T value;
    if (!node.getParam(name, value))
    {
        value = default_value;
        node.setParam(name, value);
    }
    return value;
}

bool parseXmlRpcDouble(const XmlRpc::XmlRpcValue &raw,
                       double &value)
{
    switch (raw.getType())
    {
    case XmlRpc::XmlRpcValue::TypeDouble:
        value = static_cast<double>(raw);
        return std::isfinite(value);
    case XmlRpc::XmlRpcValue::TypeInt:
        value = static_cast<int>(raw);
        return true;
    case XmlRpc::XmlRpcValue::TypeString:
    {
        const std::string text = static_cast<std::string>(raw);
        char *end = nullptr;
        value = std::strtod(text.c_str(), &end);
        return end != text.c_str() && std::isfinite(value);
    }
    default:
        return false;
    }
}

bool getNumericArrayParam(ros::NodeHandle &node,
                          const std::string &name,
                          std::vector<double> &value)
{
    XmlRpc::XmlRpcValue raw;
    if (!node.getParam(name, raw) || raw.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        return false;
    }

    value.clear();
    value.reserve(raw.size());
    for (int i = 0; i < raw.size(); ++i)
    {
        double parsed = 0.0;
        if (!parseXmlRpcDouble(raw[i], parsed))
        {
            return false;
        }
        value.push_back(parsed);
    }
    return true;
}

std::vector<double> getVectorParamOrDefault(ros::NodeHandle &node,
                                            const std::string &name,
                                            const std::vector<double> &default_value,
                                            const size_t expected_size)
{
    std::vector<double> value;
    if (!getNumericArrayParam(node, name, value) || value.size() != expected_size)
    {
        ROS_WARN_STREAM("Parameter " << name << " must be a numeric array of size "
                                      << expected_size << ". Resetting to default.");
        value = default_value;
        node.setParam(name, value);
    }
    return value;
}

std::vector<double> getDoubleVectorParamOrDefault(ros::NodeHandle &node,
                                                  const std::string &name,
                                                  const std::vector<double> &default_value)
{
    std::vector<double> value;
    if (!getNumericArrayParam(node, name, value) || value.empty())
    {
        ROS_WARN_STREAM("Parameter " << name << " must be a non-empty numeric array. Resetting to default.");
        value = default_value;
        node.setParam(name, value);
    }
    return value;
}

Eigen::Matrix<double, 6, 1> toVec6(const std::vector<double> &values)
{
    Eigen::Matrix<double, 6, 1> out;
    out << values[0], values[1], values[2], values[3], values[4], values[5];
    return out;
}

double getPositiveParamOrDefault(ros::NodeHandle &node,
                                 const std::string &name,
                                 const double default_value)
{
    double value = getParamOrDefault<double>(node, name, default_value);
    if (!std::isfinite(value) || value <= 0.0)
    {
        ROS_WARN_STREAM("Parameter " << name << " must be positive. Resetting to " << default_value);
        value = default_value;
        node.setParam(name, value);
    }
    return value;
}

double getNonNegativeParamOrDefault(ros::NodeHandle &node,
                                    const std::string &name,
                                    const double default_value)
{
    double value = getParamOrDefault<double>(node, name, default_value);
    if (!std::isfinite(value) || value < 0.0)
    {
        ROS_WARN_STREAM("Parameter " << name << " must be non-negative. Resetting to " << default_value);
        value = default_value;
        node.setParam(name, value);
    }
    return value;
}

Eigen::Matrix3d rpyDegToRot(const double roll_deg,
                            const double pitch_deg,
                            const double yaw_deg)
{
    const double roll = roll_deg * M_PI / 180.0;
    const double pitch = pitch_deg * M_PI / 180.0;
    const double yaw = yaw_deg * M_PI / 180.0;
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

geometry_msgs::Point toPoint(const Eigen::Vector3d &p)
{
    geometry_msgs::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    return point;
}

geometry_msgs::Vector3 toVector3(const Eigen::Vector3d &v)
{
    geometry_msgs::Vector3 vector;
    vector.x = v.x();
    vector.y = v.y();
    vector.z = v.z();
    return vector;
}

geometry_msgs::Quaternion toQuaternionMsg(const Eigen::Vector4d &quat_vec)
{
    Eigen::Quaterniond quat(quat_vec(0), quat_vec(1), quat_vec(2), quat_vec(3));
    quat.normalize();

    geometry_msgs::Quaternion msg;
    msg.x = quat.x();
    msg.y = quat.y();
    msg.z = quat.z();
    msg.w = quat.w();
    return msg;
}

Eigen::Vector3d transformLocal(const Eigen::Matrix<double, 3, 4> &pose,
                               const Eigen::Vector3d &local)
{
    return pose.leftCols<3>() * local + pose.rightCols<1>();
}

void setMarkerHeader(visualization_msgs::Marker &marker,
                     const std::string &frame_id,
                     const std::string &ns,
                     const int id,
                     const int type)
{
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = type;
    marker.pose.orientation.w = 1.0;
}

void setColor(visualization_msgs::Marker &marker,
              const double r,
              const double g,
              const double b,
              const double a)
{
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

Eigen::RowVector4d makeHalfspaceRow(const Eigen::Vector3d &normal,
                                    const Eigen::Vector3d &point)
{
    Eigen::RowVector4d row;
    row.head<3>() = normal.transpose();
    row(3) = -normal.dot(point);
    return row;
}
} // namespace

struct GateFrameCfg
{
    double outer_width_m = 0.82;
    double outer_length_m = 0.30;
    double inner_width_m = 0.64;
    double inner_length_m = 0.22;
    double depth_m = 0.04;
    double depth_margin_m = 0.02;

    Eigen::Vector3d innerSize() const
    {
        return Eigen::Vector3d(depth_m, inner_width_m, inner_length_m);
    }

    Eigen::Vector3d outerMargin() const
    {
        return Eigen::Vector3d(depth_margin_m,
                               0.5 * (outer_width_m - inner_width_m),
                               0.5 * (outer_length_m - inner_length_m));
    }
};

struct UavCollisionBoxCfg
{
    double width_m = 0.34;
    double depth_m = 0.34;
    double height_m = 0.13;

    double horizontalHalfLen() const
    {
        return 0.5 * std::max(width_m, depth_m);
    }

    double verticalHalfLen() const
    {
        return 0.5 * height_m;
    }
};

struct GatePlannerConfig
{
    std::string frameId;
    std::string targetTopic;
    std::string gateMarkerTopic;
    std::string mapBoxMarkerTopic;
    std::string corridorMarkerTopic;
    std::string trajMarkerTopic;
    std::string trajPathTopic;
    std::string positionCommandTopic;
    std::string so3CommandTopic;
    std::string vehicleMarkerTopic;
    std::string vehicleEllipsoidsTopic;
    std::string startGoalMarkerTopic;
    bool useHolePolygonCorridor;
    bool requireHolePolygonCorridor;
    std::string holePolygonTopic;
    int holePolygonMinVertices;
    bool useHolePolygonTxtFile;
    std::string holePolygonTxtPath;
    bool lockHolePolygonAfterFirstValid;
    bool useBodyLocalizationStart;
    bool fallbackToStartPosWhenNoBodyOdom;
    std::string bodyOdomTopic;
    bool autoPlanWithMirroredGoal;
    std::string mirrorGoalMode;
    double autoReplanMinInterval;
    double minStartReplanShift;
    double minGoalReplanShift;
    double minHoleCenterReplanShift;
    double minHoleNormalReplanAngleDeg;
    bool autoAlignMapBoxToStartGoal;
    Eigen::Vector3d mapBoxAutoPadding;

    Eigen::Matrix<double, 6, 1> mapBox;
    Eigen::Matrix<double, 3, 4> gatePose;
    GateFrameCfg gateFrame;
    UavCollisionBoxCfg uavCollisionBox;
    Eigen::Vector3d gateInnerSize;
    Eigen::Vector3d gateOuterMargin;
    Eigen::Vector3d startPos;

    int gateCount;
    double gateSpacing;
    std::vector<double> gateRollDegs;
    std::vector<double> gatePitchDegs;
    std::vector<double> gateYawDegs;
    double goalZ;
    double gateTunnelHalfDepth;
    bool updateStartAfterPlan;
    bool loopReplay;
    bool publishCommands;
    double replayYaw;
    double commandRate;
    int trajSamples;
    int vehicleEllipsoidSamples;
    double vehicleEllipsoidAlpha;

    double safeMargin;
    double weightT;
    double lengthPerPiece;
    double initialSpeedFactor;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    Eigen::VectorXd penaltyWeights;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;
    double appendedDuration;
    double trajVizWidth;

    explicit GatePlannerConfig(ros::NodeHandle &node)
    {
        frameId = getParamOrDefault<std::string>(node, "FrameId", "odom");
        targetTopic = getParamOrDefault<std::string>(node, "TargetTopic", "/move_base_simple/goal");
        gateMarkerTopic = getParamOrDefault<std::string>(node, "GateMarkerTopic", "/gate_planner/gate");
        mapBoxMarkerTopic = getParamOrDefault<std::string>(node, "MapBoxMarkerTopic", "/gate_planner/map_box");
        corridorMarkerTopic = getParamOrDefault<std::string>(node, "CorridorMarkerTopic", "/gate_planner/corridor");
        trajMarkerTopic = getParamOrDefault<std::string>(node, "TrajectoryMarkerTopic", "/gate_planner/trajectory_marker");
        trajPathTopic = getParamOrDefault<std::string>(node, "TrajectoryPathTopic", "/gate_planner/trajectory_path");
        positionCommandTopic = getParamOrDefault<std::string>(node, "PositionCommandTopic", "/gate_planner/position_command");
        so3CommandTopic = getParamOrDefault<std::string>(node, "SO3CommandTopic", "/gate_planner/so3_command");
        vehicleMarkerTopic = getParamOrDefault<std::string>(node, "VehicleMarkerTopic", "/gate_planner/vehicle");
        vehicleEllipsoidsTopic = getParamOrDefault<std::string>(node, "VehicleEllipsoidsTopic", "/gate_planner/vehicle_ellipsoids");
        startGoalMarkerTopic = getParamOrDefault<std::string>(node, "StartGoalMarkerTopic", "/gate_planner/start_goal");
        useHolePolygonCorridor = getParamOrDefault<bool>(node, "UseHolePolygonCorridor", false);
        requireHolePolygonCorridor = getParamOrDefault<bool>(node, "RequireHolePolygonCorridor", false);
        holePolygonTopic = getParamOrDefault<std::string>(node, "HolePolygonTopic", "/polygon_hole_step_viz/hole_polygon_cloud");
        holePolygonMinVertices = std::max(3, getParamOrDefault<int>(node, "HolePolygonMinVertices", 4));
        useHolePolygonTxtFile = getParamOrDefault<bool>(node, "UseHolePolygonTxtFile", false);
        holePolygonTxtPath = getParamOrDefault<std::string>(node, "HolePolygonTxtPath", "/tmp/hole_polygon_latest.txt");
        lockHolePolygonAfterFirstValid = getParamOrDefault<bool>(node, "LockHolePolygonAfterFirstValid", false);
        useBodyLocalizationStart = getParamOrDefault<bool>(node, "UseBodyLocalizationStart", false);
        fallbackToStartPosWhenNoBodyOdom = getParamOrDefault<bool>(node, "FallbackToStartPosWhenNoBodyOdom", true);
        bodyOdomTopic = getParamOrDefault<std::string>(node, "BodyOdomTopic", "/aft_mapped_to_init");
        autoPlanWithMirroredGoal = getParamOrDefault<bool>(node, "AutoPlanWithMirroredGoal", false);
        mirrorGoalMode = getParamOrDefault<std::string>(node, "MirrorGoalMode", "plane");
        autoReplanMinInterval = getNonNegativeParamOrDefault(node, "AutoReplanMinInterval", 1.0);
        minStartReplanShift = getNonNegativeParamOrDefault(node, "MinStartReplanShift", 0.05);
        minGoalReplanShift = getNonNegativeParamOrDefault(node, "MinGoalReplanShift", 0.05);
        minHoleCenterReplanShift = getNonNegativeParamOrDefault(node, "MinHoleCenterReplanShift", 0.03);
        minHoleNormalReplanAngleDeg = getNonNegativeParamOrDefault(node, "MinHoleNormalReplanAngleDeg", 3.0);
        autoAlignMapBoxToStartGoal = getParamOrDefault<bool>(node, "AutoAlignMapBoxToStartGoal", false);
        const auto map_box_auto_padding = getVectorParamOrDefault(node, "MapBoxAutoPadding",
                                                                  {0.30, 0.30, 0.30}, 3);
        mapBoxAutoPadding << getNonNegativeParamOrDefault(node, "MapBoxAutoPadX", map_box_auto_padding[0]),
            getNonNegativeParamOrDefault(node, "MapBoxAutoPadY", map_box_auto_padding[1]),
            getNonNegativeParamOrDefault(node, "MapBoxAutoPadZ", map_box_auto_padding[2]);
        node.setParam("MapBoxAutoPadding", std::vector<double>{
                                              mapBoxAutoPadding.x(), mapBoxAutoPadding.y(), mapBoxAutoPadding.z()});

        mapBox = toVec6(getVectorParamOrDefault(node, "MapBox",
                                                {-3.0, -2.0, 0.35, 3.0, 2.0, 2.4}, 6));

        const auto gate_pose = getVectorParamOrDefault(node, "GatePose",
                                                       {0.0, 0.0, 0.0, 0.0, 0.0, 1.2}, 6);
        const double gate_roll_deg = getParamOrDefault<double>(node, "GateRollDeg", gate_pose[0]);
        const double gate_pitch_deg = getParamOrDefault<double>(node, "GatePitchDeg", gate_pose[1]);
        const double gate_yaw_deg = getParamOrDefault<double>(node, "GateYawDeg", gate_pose[2]);
        const double gate_x = getParamOrDefault<double>(node, "GateX", gate_pose[3]);
        const double gate_y = getParamOrDefault<double>(node, "GateY", gate_pose[4]);
        const double gate_z = getParamOrDefault<double>(node, "GateZ", gate_pose[5]);
        gatePose.leftCols<3>() = rpyDegToRot(gate_roll_deg, gate_pitch_deg, gate_yaw_deg);
        gatePose.rightCols<1>() << gate_x, gate_y, gate_z;
        node.setParam("GatePose", std::vector<double>{gate_roll_deg, gate_pitch_deg, gate_yaw_deg,
                                                       gate_x, gate_y, gate_z});

        gateFrame.outer_width_m = getPositiveParamOrDefault(node, "GateOuterWidth", gateFrame.outer_width_m);
        gateFrame.outer_length_m = getPositiveParamOrDefault(node, "GateOuterLength", gateFrame.outer_length_m);
        gateFrame.inner_width_m = getPositiveParamOrDefault(node, "GateInnerWidth", gateFrame.inner_width_m);
        gateFrame.inner_length_m = getPositiveParamOrDefault(node, "GateInnerLength", gateFrame.inner_length_m);
        gateFrame.depth_m = getPositiveParamOrDefault(node, "GateDepth", gateFrame.depth_m);
        gateFrame.depth_margin_m = getNonNegativeParamOrDefault(node, "GateDepthMargin", gateFrame.depth_margin_m);
        if (gateFrame.inner_width_m > gateFrame.outer_width_m)
        {
            ROS_WARN_STREAM("GateInnerWidth exceeds GateOuterWidth. Clamping inner width to "
                            << gateFrame.outer_width_m);
            gateFrame.inner_width_m = gateFrame.outer_width_m;
            node.setParam("GateInnerWidth", gateFrame.inner_width_m);
        }
        if (gateFrame.inner_length_m > gateFrame.outer_length_m)
        {
            ROS_WARN_STREAM("GateInnerLength exceeds GateOuterLength. Clamping inner length to "
                            << gateFrame.outer_length_m);
            gateFrame.inner_length_m = gateFrame.outer_length_m;
            node.setParam("GateInnerLength", gateFrame.inner_length_m);
        }
        gateInnerSize = gateFrame.innerSize();
        gateOuterMargin = gateFrame.outerMargin();
        node.setParam("GateInnerSize", std::vector<double>{gateInnerSize.x(), gateInnerSize.y(), gateInnerSize.z()});
        node.setParam("GateOuterMargin", std::vector<double>{gateOuterMargin.x(), gateOuterMargin.y(), gateOuterMargin.z()});
        gateCount = std::max(1, getParamOrDefault<int>(node, "GateCount", 1));
        gateSpacing = getPositiveParamOrDefault(node, "GateSpacing", 0.85);
        gateRollDegs = getDoubleVectorParamOrDefault(node, "GateRollDegs", {gate_roll_deg});
        gatePitchDegs = getDoubleVectorParamOrDefault(node, "GatePitchDegs", {gate_pitch_deg});
        gateYawDegs = getDoubleVectorParamOrDefault(node, "GateYawDegs", {gate_yaw_deg});
        const auto start_pos = getVectorParamOrDefault(node, "StartPos",
                                                       {-2.2, 0.0, 1.2}, 3);
        startPos << getParamOrDefault<double>(node, "StartX", start_pos[0]),
            getParamOrDefault<double>(node, "StartY", start_pos[1]),
            getParamOrDefault<double>(node, "StartZ", start_pos[2]);
        node.setParam("StartPos", std::vector<double>{startPos.x(), startPos.y(), startPos.z()});

        goalZ = getParamOrDefault<double>(node, "GoalZ", startPos.z());
        gateTunnelHalfDepth = getParamOrDefault<double>(node, "GateTunnelHalfDepth", 0.60);
        updateStartAfterPlan = getParamOrDefault<bool>(node, "UpdateStartAfterPlan", false);
        loopReplay = getParamOrDefault<bool>(node, "LoopReplay", true);
        publishCommands = getParamOrDefault<bool>(node, "PublishCommands", true);
        replayYaw = getParamOrDefault<double>(node, "ReplayYaw", 0.0);
        commandRate = getPositiveParamOrDefault(node, "CommandRate", 100.0);
        trajSamples = getParamOrDefault<int>(node, "TrajectorySamples", 240);
        vehicleEllipsoidSamples = getParamOrDefault<int>(node, "VehicleEllipsoidSamples", 24);
        vehicleEllipsoidAlpha = getParamOrDefault<double>(node, "VehicleEllipsoidAlpha", 0.26);

        uavCollisionBox.width_m = getPositiveParamOrDefault(node, "UavCollisionBoxWidth", uavCollisionBox.width_m);
        uavCollisionBox.depth_m = getPositiveParamOrDefault(node, "UavCollisionBoxDepth", uavCollisionBox.depth_m);
        uavCollisionBox.height_m = getPositiveParamOrDefault(node, "UavCollisionBoxHeight", uavCollisionBox.height_m);
        safeMargin = getNonNegativeParamOrDefault(node, "SafeMargin", 0.02);
        weightT = getPositiveParamOrDefault(node, "WeightT", 20.0);
        lengthPerPiece = getPositiveParamOrDefault(node, "LengthPerPiece", 0.60);
        initialSpeedFactor = getPositiveParamOrDefault(node, "InitialSpeedFactor", 0.8);
        maxVelMag = getPositiveParamOrDefault(node, "MaxVelMag", 3.0);
        maxBdrMag = getPositiveParamOrDefault(node, "MaxBdrMag", 6.0);
        maxTiltAngle = getPositiveParamOrDefault(node, "MaxTiltAngle", 1.20);
        minThrust = getPositiveParamOrDefault(node, "MinThrust", 2.0);
        maxThrust = getPositiveParamOrDefault(node, "MaxThrust", 12.0);
        vehicleMass = getPositiveParamOrDefault(node, "VehicleMass", 0.61);
        gravAcc = getParamOrDefault<double>(node, "GravAcc", 9.81);
        horizDrag = getNonNegativeParamOrDefault(node, "HorizDrag", 0.70);
        vertDrag = getNonNegativeParamOrDefault(node, "VertDrag", 0.80);
        parasDrag = getNonNegativeParamOrDefault(node, "ParasDrag", 0.01);
        speedEps = getPositiveParamOrDefault(node, "SpeedEps", 0.0001);
        if (maxThrust <= minThrust)
        {
            ROS_WARN_STREAM("MaxThrust must be larger than MinThrust. Resetting MaxThrust to "
                            << minThrust + 1.0);
            maxThrust = minThrust + 1.0;
            node.setParam("MaxThrust", maxThrust);
        }

        const auto penalty = getVectorParamOrDefault(node, "ChiVec",
                                                     {1.0e6, 1.0e4, 1.0e4, 1.0e4, 1.0e5}, 5);
        penaltyWeights.resize(5);
        penaltyWeights << penalty[0], penalty[1], penalty[2], penalty[3], penalty[4];

        smoothingEps = getPositiveParamOrDefault(node, "SmoothingEps", 1.0e-2);
        integralIntervs = std::max(1, getParamOrDefault<int>(node, "IntegralIntervs", 16));
        relCostTol = getPositiveParamOrDefault(node, "RelCostTol", 1.0e-5);
        appendedDuration = getParamOrDefault<double>(node, "AppendedDuration", 1.0);
        trajVizWidth = getParamOrDefault<double>(node, "TrajVizWidth", 0.04);

        gateTunnelHalfDepth = std::max(gateTunnelHalfDepth,
                                       0.5 * gateInnerSize.x() + gateOuterMargin.x() + 1.0e-3);
        trajSamples = std::max(trajSamples, 2);
        vehicleEllipsoidSamples = std::max(vehicleEllipsoidSamples, 2);
        vehicleEllipsoidAlpha = std::max(0.0, std::min(1.0, vehicleEllipsoidAlpha));
    }
};

class GatePlanner
{
public:
    GatePlanner(const GatePlannerConfig &config,
                ros::NodeHandle &node)
        : config_(config),
          node_(node),
          currentStart_(config.startPos),
          activeMapBox_(config.mapBox),
          rosPtr_(std::make_shared<ros_interface::Ros1Interface>(node_)),
          trajOptCfg_(buildTrajectoryOptimizerConfig()),
          trajOpt_(std::make_unique<traj_opt::SE3AggressiveTrajOpt>(trajOptCfg_, rosPtr_)),
          hasTraj_(false),
          trajStamp_(0.0),
          trajId_(0)
    {
        targetSub_ = node_.subscribe(config_.targetTopic, 1,
                                     &GatePlanner::targetCallback, this,
                                     ros::TransportHints().tcpNoDelay());
        if (config_.useHolePolygonCorridor)
        {
            holePolygonSub_ = node_.subscribe(config_.holePolygonTopic, 1,
                                              &GatePlanner::holePolygonCallback, this,
                                              ros::TransportHints().tcpNoDelay());
        }
        if (config_.useBodyLocalizationStart)
        {
            bodyOdomSub_ = node_.subscribe(config_.bodyOdomTopic, 5,
                                           &GatePlanner::bodyOdomCallback, this,
                                           ros::TransportHints().tcpNoDelay());
        }

        gateMarkerPub_ = node_.advertise<visualization_msgs::Marker>(config_.gateMarkerTopic, 1, true);
        mapBoxMarkerPub_ = node_.advertise<visualization_msgs::Marker>(config_.mapBoxMarkerTopic, 1, true);
        corridorMarkerPub_ = node_.advertise<visualization_msgs::MarkerArray>(config_.corridorMarkerTopic, 1, true);
        trajMarkerPub_ = node_.advertise<visualization_msgs::Marker>(config_.trajMarkerTopic, 1, true);
        trajPathPub_ = node_.advertise<nav_msgs::Path>(config_.trajPathTopic, 1, true);
        positionCommandPub_ = node_.advertise<quadrotor_msgs::PositionCommand>(config_.positionCommandTopic, 10);
        so3CommandPub_ = node_.advertise<quadrotor_msgs::SO3Command>(config_.so3CommandTopic, 10);
        vehicleMarkerPub_ = node_.advertise<visualization_msgs::Marker>(config_.vehicleMarkerTopic, 1);
        vehicleEllipsoidsPub_ = node_.advertise<visualization_msgs::MarkerArray>(config_.vehicleEllipsoidsTopic, 1, true);
        startGoalMarkerPub_ = node_.advertise<visualization_msgs::Marker>(config_.startGoalMarkerTopic, 4, true);

        vizTimer_ = node_.createTimer(ros::Duration(0.05), &GatePlanner::timerCallback, this);
        commandTimer_ = node_.createTimer(ros::Duration(1.0 / config_.commandRate),
                                          &GatePlanner::commandTimerCallback, this);
        publishStaticMarkers();
        publishStartMarker(currentStart_);

        ROS_INFO_STREAM("Gate planner ready. Use RViz 2D Nav Goal on the opposite side of the gate. "
                        << makeGoalHint(currentStart_)
                        << " RViz 2D Pose Estimate is ignored; edit StartPos to move the start. "
                        << "The clicked goal z is ignored and GoalZ = " << config_.goalZ << " is used."
                        << " use_hole_polygon_corridor=" << (config_.useHolePolygonCorridor ? "true" : "false")
                        << " use_body_localization_start=" << (config_.useBodyLocalizationStart ? "true" : "false")
                        << " auto_plan_with_mirrored_goal=" << (config_.autoPlanWithMirroredGoal ? "true" : "false"));
    }

private:
    traj_opt::Config buildTrajectoryOptimizerConfig() const
    {
        traj_opt::Config cfg;
        cfg.opt_accuracy = std::max(1.0e-8, config_.relCostTol);
        cfg.smooth_eps = std::max(1.0e-12, config_.smoothingEps);
        cfg.integral_reso = std::max(1, config_.integralIntervs);
        cfg.quadrotot_flatness.reset(config_.vehicleMass,
                                    config_.gravAcc,
                                    config_.horizDrag,
                                    config_.vertDrag,
                                    config_.parasDrag,
                                    config_.speedEps);
        cfg.mass = config_.vehicleMass;
        cfg.dh = config_.horizDrag;
        cfg.dv = config_.vertDrag;
        cfg.grav = config_.gravAcc;
        cfg.cp = config_.parasDrag;
        cfg.v_eps = config_.speedEps;
        return cfg;
    }

    GatePlannerConfig config_;
    ros::NodeHandle node_;
    ros_interface::RosInterface::Ptr rosPtr_;
    traj_opt::Config trajOptCfg_;
    std::unique_ptr<traj_opt::SE3AggressiveTrajOpt> trajOpt_;
    ros::Subscriber targetSub_;
    ros::Subscriber holePolygonSub_;
    ros::Subscriber bodyOdomSub_;
    ros::Publisher gateMarkerPub_;
    ros::Publisher mapBoxMarkerPub_;
    ros::Publisher corridorMarkerPub_;
    ros::Publisher trajMarkerPub_;
    ros::Publisher trajPathPub_;
    ros::Publisher positionCommandPub_;
    ros::Publisher so3CommandPub_;
    ros::Publisher vehicleMarkerPub_;
    ros::Publisher vehicleEllipsoidsPub_;
    ros::Publisher startGoalMarkerPub_;
    ros::Timer vizTimer_;
    ros::Timer commandTimer_;

    Eigen::Vector3d currentStart_;
    Eigen::Matrix<double, 6, 1> activeMapBox_;
    geometry_utils::Trajectory lastTraj_;
    bool hasTraj_;
    double trajStamp_;
    uint32_t trajId_;
    bool hasBodyStart_ = false;
    double lastAutoPlanTime_ = -1.0;
    bool hasLastPlanSignature_ = false;
    Eigen::Vector3d lastPlanStart_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastPlanGoal_ = Eigen::Vector3d::Zero();
    bool lastPlanHadHole_ = false;
    Eigen::Vector3d lastPlanHoleCenter_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d lastPlanHoleNormal_ = Eigen::Vector3d::UnitX();
    int lastPlanHoleVertices_ = 0;

    struct HolePolygonData
    {
        bool valid = false;
        std::vector<Eigen::Vector3d> vertices;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
    } holePolygon_;

    static std::string trim(const std::string &s)
    {
        const auto b = s.find_first_not_of(" \t\r\n");
        if (b == std::string::npos) return "";
        const auto e = s.find_last_not_of(" \t\r\n");
        return s.substr(b, e - b + 1);
    }

    bool loadHolePolygonFromTxt(std::string *error = nullptr)
    {
        if (!config_.useHolePolygonTxtFile)
        {
            if (error) *error = "txt loading disabled";
            return false;
        }
        std::ifstream ifs(config_.holePolygonTxtPath);
        if (!ifs.is_open())
        {
            if (error) *error = "cannot open txt file";
            return false;
        }

        std::string line;
        std::string file_frame;
        Eigen::Vector3d file_center = Eigen::Vector3d::Zero();
        bool has_file_center = false;
        std::vector<Eigen::Vector3d> vertices;
        size_t expected_vertices = 0;
        bool in_vertex_block = false;
        while (std::getline(ifs, line))
        {
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            if (line.rfind("frame_id:", 0) == 0)
            {
                file_frame = trim(line.substr(std::string("frame_id:").size()));
                continue;
            }
            if (line.rfind("center:", 0) == 0)
            {
                std::istringstream iss(trim(line.substr(std::string("center:").size())));
                double cx = 0.0, cy = 0.0, cz = 0.0;
                if (iss >> cx >> cy >> cz)
                {
                    if (std::isfinite(cx) && std::isfinite(cy) && std::isfinite(cz))
                    {
                        file_center = Eigen::Vector3d(cx, cy, cz);
                        has_file_center = true;
                    }
                }
                continue;
            }
            if (line.rfind("vertex_count:", 0) == 0)
            {
                std::istringstream iss(trim(line.substr(std::string("vertex_count:").size())));
                size_t n = 0;
                if (iss >> n) expected_vertices = n;
                continue;
            }
            if (line == "vertices_xyz:")
            {
                in_vertex_block = true;
                continue;
            }
            if (!in_vertex_block) continue;

            std::istringstream iss(line);
            double idx = 0.0, x = 0.0, y = 0.0, z = 0.0;
            if ((iss >> idx >> x >> y >> z) || (iss.clear(), iss.str(line), (iss >> x >> y >> z)))
            {
                vertices.emplace_back(x, y, z);
            }
        }

        if (!file_frame.empty() && file_frame != config_.frameId)
        {
            if (error) *error = "frame mismatch in txt";
            return false;
        }
        if (expected_vertices > 0 && vertices.size() > expected_vertices)
        {
            vertices.resize(expected_vertices);
        }
        if (vertices.size() < static_cast<size_t>(config_.holePolygonMinVertices))
        {
            if (error) *error = "txt vertices too few";
            return false;
        }
        if ((vertices.front() - vertices.back()).norm() < 1.0e-4)
        {
            vertices.pop_back();
        }
        if (vertices.size() < static_cast<size_t>(config_.holePolygonMinVertices))
        {
            if (error) *error = "txt vertices too few after dedup";
            return false;
        }

        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        if (has_file_center)
        {
            center = file_center;
        }
        else
        {
            for (const auto &p : vertices) center += p;
            center /= static_cast<double>(vertices.size());
        }

        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            const Eigen::Vector3d a = vertices[i] - center;
            const Eigen::Vector3d b = vertices[(i + 1) % vertices.size()] - center;
            normal += a.cross(b);
        }
        if (normal.norm() < 1.0e-6)
        {
            if (error) *error = "txt polygon normal invalid";
            return false;
        }
        normal.normalize();

        const Eigen::Vector3d pose_normal = config_.gatePose.leftCols<3>() * Eigen::Vector3d::UnitX();
        if (normal.dot(pose_normal) < 0.0)
        {
            normal = -normal;
            std::reverse(vertices.begin(), vertices.end());
        }
        for (auto &p : vertices)
        {
            p = p - normal.dot(p - center) * normal;
        }

        holePolygon_.valid = true;
        holePolygon_.vertices = vertices;
        holePolygon_.center = center;
        holePolygon_.normal = normal;
        ROS_INFO_STREAM("Loaded hole polygon from txt: " << config_.holePolygonTxtPath
                        << " vertices=" << holePolygon_.vertices.size()
                        << " center=[" << holePolygon_.center.transpose() << "]");
        return true;
    }

    Eigen::Vector3d toGateLocal(const Eigen::Vector3d &point) const
    {
        const Eigen::Matrix3d rot = config_.gatePose.leftCols<3>();
        const Eigen::Vector3d center = config_.gatePose.rightCols<1>();
        return rot.transpose() * (point - center);
    }

    Eigen::Vector3d gateCenter() const
    {
        if (config_.useHolePolygonCorridor && holePolygon_.valid)
        {
            return holePolygon_.center;
        }
        return config_.gatePose.rightCols<1>();
    }

    Eigen::Vector3d gateNormal() const
    {
        if (config_.useHolePolygonCorridor && holePolygon_.valid)
        {
            return holePolygon_.normal;
        }
        return config_.gatePose.leftCols<3>() * Eigen::Vector3d::UnitX();
    }

    double gateStackHalfSpan() const
    {
        return 0.5 * static_cast<double>(std::max(config_.gateCount - 1, 0)) * config_.gateSpacing;
    }

    double gateAngleAt(const std::vector<double> &angles,
                       const int index) const
    {
        if (angles.empty())
        {
            return 0.0;
        }
        if (index < static_cast<int>(angles.size()))
        {
            return angles[index];
        }
        return angles.back();
    }

    Eigen::Matrix3d gateRotationAt(const int index) const
    {
        return rpyDegToRot(gateAngleAt(config_.gateRollDegs, index),
                           gateAngleAt(config_.gatePitchDegs, index),
                           gateAngleAt(config_.gateYawDegs, index));
    }

    std::vector<Eigen::Matrix<double, 3, 4>> gatePosesAll() const
    {
        std::vector<Eigen::Matrix<double, 3, 4>> poses;
        poses.reserve(config_.gateCount);
        const Eigen::Vector3d center = gateCenter();
        const Eigen::Vector3d stack_normal = gateNormal().normalized();
        const double half_span = gateStackHalfSpan();
        for (int i = 0; i < config_.gateCount; ++i)
        {
            const double offset = -half_span + static_cast<double>(i) * config_.gateSpacing;
            Eigen::Matrix<double, 3, 4> pose;
            pose.leftCols<3>() = gateRotationAt(i);
            pose.rightCols<1>() = center + offset * stack_normal;
            poses.push_back(pose);
        }
        return poses;
    }

    std::vector<Eigen::Matrix<double, 3, 4>> gatePosesOrdered(const Eigen::Vector3d &start,
                                                              const Eigen::Vector3d &goal) const
    {
        std::vector<Eigen::Matrix<double, 3, 4>> poses = gatePosesAll();
        const Eigen::Vector3d center = gateCenter();
        const Eigen::Vector3d stack_normal = gateNormal().normalized();
        if (stack_normal.dot(start - center) > stack_normal.dot(goal - center))
        {
            std::reverse(poses.begin(), poses.end());
        }
        return poses;
    }

    std::vector<Eigen::Vector3d> gateCentersAll() const
    {
        std::vector<Eigen::Vector3d> centers;
        const auto poses = gatePosesAll();
        centers.reserve(poses.size());
        for (const auto &pose : poses)
        {
            centers.push_back(pose.rightCols<1>());
        }
        return centers;
    }

    Eigen::Vector3d makeMirroredGoal(const Eigen::Vector3d &start) const
    {
        const Eigen::Vector3d center = gateCenter();
        const Eigen::Vector3d normal = gateNormal().normalized();
        Eigen::Vector3d goal = start;
        if (config_.mirrorGoalMode == "center")
        {
            goal = 2.0 * center - start;
            const double signed_dist = normal.dot(start - center);
            if (config_.gateCount > 1)
            {
                goal += 2.0 * gateStackHalfSpan() * (signed_dist < 0.0 ? normal : -normal);
            }
        }
        else
        {
            const double signed_dist = normal.dot(start - center);
            goal = start - 2.0 * signed_dist * normal;
            if (config_.gateCount > 1)
            {
                goal += 2.0 * gateStackHalfSpan() * (signed_dist < 0.0 ? normal : -normal);
            }
        }
        return goal;
    }

    static double angleDegBetween(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
    {
        if (a.norm() < 1.0e-9 || b.norm() < 1.0e-9) return 180.0;
        const double c = std::max(-1.0, std::min(1.0, std::abs(a.normalized().dot(b.normalized()))));
        return std::acos(c) * 180.0 / M_PI;
    }

    bool hasSignificantReplanChange(const Eigen::Vector3d &start,
                                    const Eigen::Vector3d &goal) const
    {
        if (!hasLastPlanSignature_) return true;

        if ((start - lastPlanStart_).norm() >= config_.minStartReplanShift) return true;
        if ((goal - lastPlanGoal_).norm() >= config_.minGoalReplanShift) return true;

        const bool hole_now = config_.useHolePolygonCorridor && holePolygon_.valid;
        if (hole_now != lastPlanHadHole_) return true;
        if (hole_now)
        {
            if ((holePolygon_.center - lastPlanHoleCenter_).norm() >= config_.minHoleCenterReplanShift) return true;
            if (angleDegBetween(holePolygon_.normal, lastPlanHoleNormal_) >= config_.minHoleNormalReplanAngleDeg) return true;
            if (static_cast<int>(holePolygon_.vertices.size()) != lastPlanHoleVertices_) return true;
        }
        return false;
    }

    void updateLastPlanSignature(const Eigen::Vector3d &start,
                                 const Eigen::Vector3d &goal)
    {
        hasLastPlanSignature_ = true;
        lastPlanStart_ = start;
        lastPlanGoal_ = goal;
        lastPlanHadHole_ = config_.useHolePolygonCorridor && holePolygon_.valid;
        if (lastPlanHadHole_)
        {
            lastPlanHoleCenter_ = holePolygon_.center;
            lastPlanHoleNormal_ = holePolygon_.normal;
            lastPlanHoleVertices_ = static_cast<int>(holePolygon_.vertices.size());
        }
        else
        {
            lastPlanHoleCenter_.setZero();
            lastPlanHoleNormal_ = Eigen::Vector3d::UnitX();
            lastPlanHoleVertices_ = 0;
        }
    }

    void maybeAutoPlanMirror(const char *reason)
    {
        if (!config_.autoPlanWithMirroredGoal) return;
        if (config_.useBodyLocalizationStart && !hasBodyStart_)
        {
            if (!config_.fallbackToStartPosWhenNoBodyOdom) return;
            ROS_WARN_THROTTLE(1.0,
                              "No body odom yet on %s, fallback to StartPos=[%.3f %.3f %.3f].",
                              config_.bodyOdomTopic.c_str(),
                              currentStart_.x(), currentStart_.y(), currentStart_.z());
        }
        if (config_.useHolePolygonCorridor && !holePolygon_.valid && config_.useHolePolygonTxtFile)
        {
            std::string err;
            loadHolePolygonFromTxt(&err);
            if (!holePolygon_.valid)
            {
                ROS_WARN_THROTTLE(2.0, "Hole polygon txt not ready: %s (%s)",
                                  config_.holePolygonTxtPath.c_str(), err.c_str());
            }
        }
        if (config_.requireHolePolygonCorridor && !holePolygon_.valid) return;
        const double now = ros::Time::now().toSec();
        if (lastAutoPlanTime_ > 0.0 && (now - lastAutoPlanTime_) < config_.autoReplanMinInterval) return;
        const Eigen::Vector3d goal = makeMirroredGoal(currentStart_);
        if (!hasSignificantReplanChange(currentStart_, goal))
        {
            ROS_INFO_THROTTLE(1.0, "Skip auto plan: start/goal/hole unchanged within thresholds.");
            return;
        }
        ROS_INFO_STREAM("Auto plan (" << reason << ") start=[" << currentStart_.transpose()
                        << "] goal(mirror)=[" << goal.transpose() << "] mode=" << config_.mirrorGoalMode);
        lastAutoPlanTime_ = now;
        if (planTo(goal))
        {
            updateLastPlanSignature(currentStart_, goal);
        }
    }

    void bodyOdomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        if (!config_.useBodyLocalizationStart) return;
        if (!msg->header.frame_id.empty() && msg->header.frame_id != config_.frameId)
        {
            ROS_WARN_THROTTLE(1.0,
                              "Body odom frame mismatch: msg frame=%s expected=%s",
                              msg->header.frame_id.c_str(), config_.frameId.c_str());
            return;
        }
        currentStart_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z);
        hasBodyStart_ = true;
        publishStartMarker(currentStart_);
        maybeAutoPlanMirror("body_odom");
    }

    void holePolygonCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        if (!config_.useHolePolygonCorridor)
        {
            return;
        }
        if (config_.lockHolePolygonAfterFirstValid && holePolygon_.valid)
        {
            return;
        }
        if (msg->header.frame_id != config_.frameId)
        {
            ROS_WARN_THROTTLE(1.0,
                              "Hole polygon frame mismatch: msg frame=%s expected=%s. "
                              "Set FrameId to match hole polygon frame.",
                              msg->header.frame_id.c_str(), config_.frameId.c_str());
            return;
        }
        if (msg->data.empty() || msg->width * msg->height < static_cast<uint32_t>(config_.holePolygonMinVertices))
        {
            ROS_WARN_THROTTLE(1.0, "Hole polygon cloud too small.");
            return;
        }

        std::vector<Eigen::Vector3d> vertices;
        vertices.reserve(msg->width * msg->height);
        try
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                vertices.emplace_back(*iter_x, *iter_y, *iter_z);
            }
        }
        catch (const std::runtime_error &e)
        {
            ROS_WARN_THROTTLE(1.0, "Failed to parse hole polygon point cloud: %s", e.what());
            return;
        }

        if (vertices.size() < static_cast<size_t>(config_.holePolygonMinVertices))
        {
            ROS_WARN_THROTTLE(1.0, "Hole polygon vertices too few: %zu", vertices.size());
            return;
        }

        if ((vertices.front() - vertices.back()).norm() < 1.0e-4)
        {
            vertices.pop_back();
        }
        if (vertices.size() < static_cast<size_t>(config_.holePolygonMinVertices))
        {
            return;
        }

        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const auto &p : vertices)
        {
            center += p;
        }
        center /= static_cast<double>(vertices.size());

        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            const Eigen::Vector3d a = vertices[i] - center;
            const Eigen::Vector3d b = vertices[(i + 1) % vertices.size()] - center;
            normal += a.cross(b);
        }
        if (normal.norm() < 1.0e-6)
        {
            ROS_WARN_THROTTLE(1.0, "Invalid hole polygon normal.");
            return;
        }
        normal.normalize();

        const Eigen::Vector3d pose_normal = config_.gatePose.leftCols<3>() * Eigen::Vector3d::UnitX();
        if (normal.dot(pose_normal) < 0.0)
        {
            normal = -normal;
            std::reverse(vertices.begin(), vertices.end());
        }

        // Project vertices back to the fitted plane to reduce boundary noise.
        for (auto &p : vertices)
        {
            p = p - normal.dot(p - center) * normal;
        }

        holePolygon_.valid = true;
        holePolygon_.vertices = vertices;
        holePolygon_.center = center;
        holePolygon_.normal = normal;
        ROS_INFO_STREAM_THROTTLE(1.0,
                                 "Updated hole polygon: vertices=" << holePolygon_.vertices.size()
                                 << " center=[" << holePolygon_.center.transpose() << "]"
                                 << " normal=[" << holePolygon_.normal.transpose() << "]"
                                 << " lock_after_first_valid=" << (config_.lockHolePolygonAfterFirstValid ? "true" : "false"));
        maybeAutoPlanMirror("hole_polygon");
    }

    double gateSideSeparation() const
    {
        return 0.5 * config_.gateInnerSize.x() + config_.gateOuterMargin.x();
    }

    std::string makeGoalHint(const Eigen::Vector3d &start) const
    {
        const Eigen::Vector3d normal = gateNormal();
        const Eigen::Vector3d center = gateCenter();
        const double start_side = normal.dot(start - center);
        const double side_sep = gateSideSeparation();
        const double half_span = gateStackHalfSpan();
        const double min_side = -half_span - side_sep;
        const double max_side = half_span + side_sep;
        const bool gate_normal_is_world_x = (normal - Eigen::Vector3d::UnitX()).norm() < 1.0e-6;
        std::ostringstream out;
        out << "Start world = [" << start.transpose() << "], signed distance to gate plane = "
            << start_side << ", gate center world = ["
            << center.transpose() << "], gate_count = " << config_.gateCount << ". ";
        if (start_side < 0.0)
        {
            out << "For this start, click a goal with gate-stack local x > " << max_side << ".";
            if (gate_normal_is_world_x)
            {
                out << " With the current unrotated gate, this is world x > " << center.x() + max_side << ".";
            }
        }
        else
        {
            out << "For this start, click a goal with gate-stack local x < " << min_side << ".";
            if (gate_normal_is_world_x)
            {
                out << " With the current unrotated gate, this is world x < " << center.x() + min_side << ".";
            }
        }
        return out.str();
    }

    Eigen::Matrix<double, 6, 1> computeActiveMapBox(const Eigen::Vector3d &start,
                                                    const Eigen::Vector3d &goal) const
    {
        if (!config_.autoAlignMapBoxToStartGoal)
        {
            return config_.mapBox;
        }

        Eigen::Matrix<double, 6, 1> box = config_.mapBox;
        const Eigen::Vector3d pad = config_.mapBoxAutoPadding;
        const auto expand_to_point = [&](const Eigen::Vector3d &p)
        {
            box(0) = std::min(box(0), p.x() - pad.x());
            box(1) = std::min(box(1), p.y() - pad.y());
            box(2) = std::min(box(2), p.z() - pad.z());
            box(3) = std::max(box(3), p.x() + pad.x());
            box(4) = std::max(box(4), p.y() + pad.y());
            box(5) = std::max(box(5), p.z() + pad.z());
        };
        expand_to_point(start);
        expand_to_point(goal);
        for (const auto &gate_center : gateCentersAll())
        {
            expand_to_point(gate_center);
        }
        return box;
    }

    std::vector<Eigen::RowVector4d> makeMapBoxHPoly(const Eigen::Matrix<double, 6, 1> &map_box_cfg) const
    {
        std::vector<Eigen::RowVector4d> map_box;
        map_box.reserve(6);
        map_box.emplace_back(Eigen::RowVector4d(-1.0, 0.0, 0.0, map_box_cfg(0)));
        map_box.emplace_back(Eigen::RowVector4d(1.0, 0.0, 0.0, -map_box_cfg(3)));
        map_box.emplace_back(Eigen::RowVector4d(0.0, -1.0, 0.0, map_box_cfg(1)));
        map_box.emplace_back(Eigen::RowVector4d(0.0, 1.0, 0.0, -map_box_cfg(4)));
        map_box.emplace_back(Eigen::RowVector4d(0.0, 0.0, -1.0, map_box_cfg(2)));
        map_box.emplace_back(Eigen::RowVector4d(0.0, 0.0, 1.0, -map_box_cfg(5)));
        return map_box;
    }

    Eigen::Matrix<double, 6, Eigen::Dynamic> toHPoly(const std::vector<Eigen::RowVector4d> &rows) const
    {
        Eigen::Matrix<double, 6, Eigen::Dynamic> hpoly(6, static_cast<Eigen::Index>(rows.size()));
        hpoly.setZero();
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(rows.size()); ++i)
        {
            const Eigen::RowVector4d &row = rows[static_cast<size_t>(i)];
            Eigen::Vector3d normal = row.head<3>().transpose();
            const double n_norm = normal.norm();
            if (n_norm < 1.0e-12)
            {
                continue;
            }
            normal /= n_norm;
            const double d = row(3);
            const Eigen::Vector3d point = -d * normal / n_norm;
            hpoly.col(i).head<3>() = normal;
            hpoly.col(i).tail<3>() = point;
        }
        return hpoly;
    }

    bool isInside(const Eigen::Matrix<double, 6, Eigen::Dynamic> &hPoly,
                  const Eigen::Vector3d &pos,
                  const double eps = 1.0e-6) const
    {
        for (int col = 0; col < hPoly.cols(); ++col)
        {
            const Eigen::Vector3d normal = hPoly.col(col).head<3>().normalized();
            if (normal.squaredNorm() < 1.0e-12)
            {
                continue;
            }
            const Eigen::Vector3d point = hPoly.col(col).tail<3>();
            if (normal.dot(pos - point) > eps)
            {
                return false;
            }
        }
        return true;
    }

    std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> buildGateCorridor(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &goal) const
    {
        const std::vector<Eigen::RowVector4d> map_box_rows = makeMapBoxHPoly(activeMapBox_);
        const Eigen::Vector3d stack_normal = gateNormal().normalized();
        const std::vector<Eigen::Matrix<double, 3, 4>> poses = gatePosesOrdered(start, goal);
        const Eigen::Vector3d travel_normal =
            stack_normal.dot(goal - start) >= 0.0 ? stack_normal : -stack_normal;
        const double side_sep = 0.5 * config_.gateInnerSize.x() + config_.gateOuterMargin.x();
        const double half_depth = config_.gateTunnelHalfDepth;

        std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> corridor;
        corridor.reserve(2 * poses.size() + 1);

        std::vector<Eigen::RowVector4d> before_rows = map_box_rows;
        before_rows.emplace_back(makeHalfspaceRow(travel_normal, poses.front().rightCols<1>() - side_sep * travel_normal));
        corridor.push_back(toHPoly(before_rows));

        for (size_t gate_idx = 0; gate_idx < poses.size(); ++gate_idx)
        {
            const auto &pose = poses[gate_idx];
            const Eigen::Vector3d center = pose.rightCols<1>();
            const Eigen::Vector3d gate_normal = pose.leftCols<3>() * Eigen::Vector3d::UnitX();
            const std::vector<Eigen::RowVector4d> side_rows = gateOpeningFacetRows(pose);

            std::vector<Eigen::RowVector4d> through_rows = map_box_rows;
            through_rows.emplace_back(makeHalfspaceRow(gate_normal, center + half_depth * gate_normal));
            through_rows.emplace_back(makeHalfspaceRow(-gate_normal, center - half_depth * gate_normal));
            for (size_t i = 0; i < side_rows.size(); ++i)
            {
                through_rows.emplace_back(side_rows[i]);
            }
            corridor.push_back(toHPoly(through_rows));

            if (gate_idx + 1 < poses.size())
            {
                std::vector<Eigen::RowVector4d> transit_rows = map_box_rows;
                transit_rows.emplace_back(makeHalfspaceRow(-travel_normal, center));
                transit_rows.emplace_back(makeHalfspaceRow(travel_normal, poses[gate_idx + 1].rightCols<1>()));
                corridor.push_back(toHPoly(transit_rows));
            }
        }

        std::vector<Eigen::RowVector4d> after_rows = map_box_rows;
        after_rows.emplace_back(makeHalfspaceRow(-travel_normal, poses.back().rightCols<1>() + side_sep * travel_normal));
        corridor.push_back(toHPoly(after_rows));
        return corridor;
    }

    std::vector<Eigen::Vector3d> buildGuidePath(const Eigen::Vector3d &start,
                                                const Eigen::Vector3d &goal) const
    {
        std::vector<Eigen::Vector3d> path;
        path.reserve(2 + static_cast<size_t>(std::max(1, config_.gateCount)));
        path.push_back(start);
        const std::vector<Eigen::Matrix<double, 3, 4>> poses = gatePosesOrdered(start, goal);
        for (const auto &pose : poses)
        {
            path.push_back(pose.rightCols<1>());
        }
        path.push_back(goal);
        return path;
    }

    std::vector<double> buildGuideTiming(const std::vector<Eigen::Vector3d> &path,
                                        const double reference_speed) const
    {
        std::vector<double> times;
        if (path.empty())
        {
            times.push_back(0.0);
            return times;
        }

        times.reserve(path.size());
        times.push_back(0.0);
        double t = 0.0;
        const double inv_speed = 1.0 / std::max(1.0e-3, reference_speed);
        for (size_t i = 1; i < path.size(); ++i)
        {
            const double ds = (path[i] - path[i - 1]).norm();
            t += std::max(0.03, ds * inv_speed);
            times.push_back(t);
        }
        return times;
    }

    double estimatePathLength(const std::vector<Eigen::Vector3d> &path) const
    {
        if (path.size() < 2)
        {
            return 0.0;
        }
        double length = 0.0;
        for (size_t i = 1; i < path.size(); ++i)
        {
            length += (path[i] - path[i - 1]).norm();
        }
        return length;
    }

    traj_opt::SE3AggressiveProblem buildAggressiveProblem(
        const Eigen::Vector3d &start,
        const Eigen::Vector3d &goal,
        const std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> &corridor) const
    {
        traj_opt::SE3AggressiveProblem problem;
        const std::vector<Eigen::Vector3d> guide_path = buildGuidePath(start, goal);
        const double reference_speed =
            std::max(0.2, config_.maxVelMag > 0.0 ? config_.maxVelMag : 1.0);
        const double path_length = estimatePathLength(guide_path);
        const double target_duration = path_length / reference_speed;

        problem.piece_num = std::max(1, config_.lengthPerPiece > 0.0
                                        ? static_cast<int>(std::ceil(path_length /
                                                                     config_.lengthPerPiece))
                                        : 1);
        problem.reference_speed = reference_speed;
        problem.min_duration = std::max(0.15, 0.5 * target_duration);
        problem.max_duration = std::max(problem.min_duration + 1.0e-3, 2.0 * target_duration);
        const auto body_half = bodyEllipsoid();
        problem.horiz_half_len = body_half.x();
        problem.vert_half_len = body_half.z();
        problem.safe_margin = config_.safeMargin;
        problem.max_vel = config_.maxVelMag;
        problem.thrust_acc_min = config_.minThrust;
        problem.thrust_acc_max = config_.maxThrust;
        problem.body_rate_max = config_.maxBdrMag;
        problem.yaw_rate_max = 3.0;
        problem.weight_time = std::max(0.0, config_.weightT);
        problem.weight_corridor =
            config_.penaltyWeights.size() > 0 ? std::abs(config_.penaltyWeights[0]) : 1.0e4;
        problem.weight_vel =
            config_.penaltyWeights.size() > 1 ? std::abs(config_.penaltyWeights[1]) : 1.0e3;
        problem.weight_thrust =
            config_.penaltyWeights.size() > 4 ? std::abs(config_.penaltyWeights[4]) : 1.0e3;
        problem.weight_body_rate =
            config_.penaltyWeights.size() > 2 ? std::abs(config_.penaltyWeights[2]) : 1.0e3;
        problem.use_yaw = false;
        problem.yaw_heading_to_velocity = true;
        problem.yaw = config_.replayYaw;
        problem.yaw_rate = 0.0;
        problem.use_corridor = !corridor.empty();
        problem.runtime_check_enable = true;
        problem.use_numeric_shape_gradient = true;
        problem.head_pvaj = general_utils::StatePVAJ::Zero();
        problem.tail_pvaj = general_utils::StatePVAJ::Zero();
        problem.head_pvaj.col(0) = start;
        problem.tail_pvaj.col(0) = goal;

        problem.guide_path.reserve(static_cast<std::size_t>(guide_path.size()));
        for (const auto &p : guide_path)
        {
            problem.guide_path.emplace_back(p);
        }
        problem.guide_t = buildGuideTiming(guide_path, problem.reference_speed);
        problem.hpolys = corridor;
        problem.piece_to_corridor.assign(static_cast<std::size_t>(problem.piece_num), 0);
        if (!corridor.empty())
        {
            for (int i = 0; i < problem.piece_num; ++i)
            {
                const double ratio = (static_cast<double>(i) + 0.5) / static_cast<double>(problem.piece_num);
                int corridor_id = static_cast<int>(std::floor(ratio * static_cast<double>(corridor.size())));
                corridor_id = std::clamp(corridor_id, 0, static_cast<int>(corridor.size()) - 1);
                problem.piece_to_corridor[static_cast<std::size_t>(i)] = corridor_id;
            }
        }
        return problem;
    }

    void targetCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        if (config_.autoPlanWithMirroredGoal)
        {
            ROS_INFO_THROTTLE(1.0, "AutoPlanWithMirroredGoal=true, ignoring 2D Nav Goal clicks.");
            return;
        }
        Eigen::Vector3d goal(msg->pose.position.x,
                             msg->pose.position.y,
                             config_.goalZ);
        planTo(goal);
    }

    bool planTo(const Eigen::Vector3d &goal)
    {
        if (config_.useHolePolygonCorridor && !holePolygon_.valid)
        {
            if (config_.requireHolePolygonCorridor)
            {
                ROS_WARN_THROTTLE(1.0,
                                  "RequireHolePolygonCorridor=true but hole polygon not ready. "
                                  "Plan is blocked.");
                return false;
            }
            ROS_WARN_THROTTLE(1.0,
                              "UseHolePolygonCorridor=true but hole polygon not ready. "
                              "Fallback to rectangle gate constraints.");
        }
        const Eigen::Vector3d start = currentStart_;
        activeMapBox_ = computeActiveMapBox(start, goal);
        const Eigen::Vector3d center = gateCenter();
        const Eigen::Vector3d normal = gateNormal();
        const double start_side = normal.dot(start - center);
        const double goal_side = normal.dot(goal - center);
        const double side_sep = gateSideSeparation();
        const double half_span = gateStackHalfSpan();
        const double min_side = -half_span - side_sep;
        const double max_side = half_span + side_sep;
        const bool start_from_negative_side = start_side < 0.0;
        if ((start_from_negative_side && start_side > min_side) ||
            (!start_from_negative_side && start_side < max_side))
        {
            ROS_WARN_STREAM("Start is too close to the gate stack to choose an entry side. "
                            << "Signed start distance = " << start_side
                            << ", required outside [" << min_side << ", " << max_side
                            << "]. Edit StartPos in gate_planning.yaml.");
            return false;
        }
        if ((start_from_negative_side && goal_side < max_side) ||
            (!start_from_negative_side && goal_side > min_side))
        {
            ROS_WARN_STREAM("2D Nav Goal must be beyond the opposite side of the gate stack from StartPos. "
                            << "Signed start/goal distances = [" << start_side
                            << ", " << goal_side << "]. "
                            << makeGoalHint(start));
            return false;
        }

        const auto corridor = buildGateCorridor(start, goal);
        corridorMarkerPub_.publish(makeCorridorMarkers(start, goal));
        if (!isInside(corridor.front(), start))
        {
            ROS_WARN_STREAM("Start is not inside the entry corridor. start = ["
                            << start.transpose() << "], gate-local = ["
                            << start_side << "]. "
                            << "Check StartPos and MapBox.");
            return false;
        }
        if (!isInside(corridor.back(), goal))
        {
            ROS_WARN_STREAM("Goal is outside the exit corridor or MapBox. goal = ["
                            << goal.transpose() << "], signed distance = ["
                            << goal_side << "]. "
                            << "Keep the 2D Nav Goal inside MapBox x/y bounds and on the opposite gate side. "
                            << makeGoalHint(start));
            return false;
        }

        const auto problem = buildAggressiveProblem(start, goal, corridor);
        geometry_utils::Trajectory traj;
        const auto tic = std::chrono::high_resolution_clock::now();
        if (!trajOpt_->optimize(problem, traj))
        {
            ROS_WARN("SE3AggressiveTrajOpt failed for this goal request.");
            return false;
        }

        const auto toc = std::chrono::high_resolution_clock::now();
        const double comp_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1.0e-3;
        const double sampled_max_vel = maxDerivNorm(traj, 1);
        const double sampled_max_acc = maxDerivNorm(traj, 2);
        ROS_INFO_STREAM("Gate trajectory solved in " << comp_ms
                                                     << " ms, corridorPolys = " << corridor.size()
                                                     << ", gateCount = " << config_.gateCount
                                                     << ", pieceNum = " << problem.piece_num
                                                     << ", trajDuration = " << traj.getTotalDuration()
                                                     << ", replayDuration = " << traj.getTotalDuration() + config_.appendedDuration
                                                     << ", maxVel = " << sampled_max_vel
                                                     << ", maxVelLimit = " << config_.maxVelMag
                                                     << ", maxAcc = " << sampled_max_acc);
        if (sampled_max_vel > config_.maxVelMag * 1.01)
        {
            ROS_WARN_STREAM("Sampled trajectory velocity exceeds MaxVelMag. "
                            << "maxVel=" << sampled_max_vel
                            << ", MaxVelMag=" << config_.maxVelMag
                            << ". This optimizer uses soft penalties; increase ChiVec[1], "
                            << "reduce WeightT, or increase total corridor length/time if a stricter limit is required.");
        }

        lastTraj_ = traj;
        hasTraj_ = true;
        ++trajId_;
        trajStamp_ = ros::Time::now().toSec();
        if (config_.updateStartAfterPlan)
        {
            currentStart_ = goal;
        }

        publishStaticMarkers();
        publishStartGoal(start, goal);
        publishTrajectory(traj);
        publishVehicleTrajectoryMarkers(traj);
        reportGateFit(traj);
        publishCommandsAt(0.0, ros::Time::now());
        return true;
    }

    void publishStaticMarkers()
    {
        gateMarkerPub_.publish(makeGateMarker());
        mapBoxMarkerPub_.publish(makeMapBoxMarker());
        publishStartMarker(currentStart_);
    }

    std::vector<Eigen::Vector3d> getThroughPolygonWorld(const Eigen::Matrix<double, 3, 4> &pose) const
    {
        std::vector<Eigen::Vector3d> poly;
        const Eigen::Vector3d center = pose.rightCols<1>();
        if (config_.useHolePolygonCorridor && holePolygon_.valid &&
            holePolygon_.vertices.size() >= static_cast<size_t>(config_.holePolygonMinVertices))
        {
            const Eigen::Vector3d polygon_shift = center - holePolygon_.center;
            poly.reserve(holePolygon_.vertices.size());
            for (const auto &vertex : holePolygon_.vertices)
            {
                poly.push_back(vertex + polygon_shift);
            }
            return poly;
        }

        const Eigen::Matrix3d rot = pose.leftCols<3>();
        const Eigen::Vector3d gate_y = rot * Eigen::Vector3d::UnitY();
        const Eigen::Vector3d gate_z = rot * Eigen::Vector3d::UnitZ();
        const double half_width = 0.5 * config_.gateInnerSize.y();
        const double half_height = 0.5 * config_.gateInnerSize.z();
        poly.push_back(center + half_width * gate_y + half_height * gate_z);
        poly.push_back(center - half_width * gate_y + half_height * gate_z);
        poly.push_back(center - half_width * gate_y - half_height * gate_z);
        poly.push_back(center + half_width * gate_y - half_height * gate_z);
        return poly;
    }

    visualization_msgs::MarkerArray makeCorridorMarkers(const Eigen::Vector3d &start,
                                                        const Eigen::Vector3d &goal) const
    {
        visualization_msgs::MarkerArray arr;
        const Eigen::Vector3d normal = gateNormal().normalized();
        const Eigen::Vector3d travel_normal =
            normal.dot(goal - start) >= 0.0 ? normal : -normal;
        const double half_depth = config_.gateTunnelHalfDepth;
        const auto poses = gatePosesOrdered(start, goal);
        if (poses.empty()) return arr;

        visualization_msgs::Marker prism;
        setMarkerHeader(prism, config_.frameId, "corridor", 0, visualization_msgs::Marker::LINE_LIST);
        setColor(prism, 0.06, 0.75, 0.95, 1.0);
        prism.scale.x = 0.02;
        auto add_edge = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
        {
            prism.points.push_back(toPoint(a));
            prism.points.push_back(toPoint(b));
        };
        for (const auto &pose : poses)
        {
            const std::vector<Eigen::Vector3d> base = getThroughPolygonWorld(pose);
            if (base.size() < 3) continue;
            const Eigen::Vector3d gate_normal = pose.leftCols<3>() * Eigen::Vector3d::UnitX();

            std::vector<Eigen::Vector3d> front;
            std::vector<Eigen::Vector3d> back;
            front.reserve(base.size());
            back.reserve(base.size());
            for (const auto &p : base)
            {
                front.push_back(p + half_depth * gate_normal);
                back.push_back(p - half_depth * gate_normal);
            }

            for (size_t i = 0; i < base.size(); ++i)
            {
                const size_t j = (i + 1) % base.size();
                add_edge(front[i], front[j]);
                add_edge(back[i], back[j]);
                add_edge(front[i], back[i]);
            }
        }
        arr.markers.push_back(prism);

        visualization_msgs::Marker centerline;
        setMarkerHeader(centerline, config_.frameId, "corridor", 1, visualization_msgs::Marker::LINE_STRIP);
        setColor(centerline, 0.98, 0.72, 0.10, 1.0);
        centerline.scale.x = 0.03;
        centerline.points.push_back(toPoint(start));
        for (const auto &pose : poses)
        {
            const Eigen::Vector3d center = pose.rightCols<1>();
            const Eigen::Vector3d gate_normal = pose.leftCols<3>() * Eigen::Vector3d::UnitX();
            const Eigen::Vector3d directed_gate_normal =
                gate_normal.dot(travel_normal) >= 0.0 ? gate_normal : -gate_normal;
            centerline.points.push_back(toPoint(center - half_depth * directed_gate_normal));
            centerline.points.push_back(toPoint(center + half_depth * directed_gate_normal));
        }
        centerline.points.push_back(toPoint(goal));
        arr.markers.push_back(centerline);
        return arr;
    }

    visualization_msgs::Marker makeGateMarker() const
    {
        visualization_msgs::Marker marker;
        setMarkerHeader(marker, config_.frameId, "gate", 0, visualization_msgs::Marker::LINE_LIST);
        setColor(marker, 0.90, 0.12, 0.08, 1.0);
        marker.scale.x = 0.025;

        const double xi = 0.5 * config_.gateInnerSize.x();
        const double yi = 0.5 * config_.gateInnerSize.y();
        const double zi = 0.5 * config_.gateInnerSize.z();
        const double xo = xi + config_.gateOuterMargin.x();
        const double yo = yi + config_.gateOuterMargin.y();
        const double zo = zi + config_.gateOuterMargin.z();

        const auto addEdge = [&](const Eigen::Matrix<double, 3, 4> &pose,
                                 const Eigen::Vector3d &a,
                                 const Eigen::Vector3d &b)
        {
            marker.points.push_back(toPoint(transformLocal(pose, a)));
            marker.points.push_back(toPoint(transformLocal(pose, b)));
        };

        const auto addBox = [&](const Eigen::Matrix<double, 3, 4> &pose,
                                const double x,
                                const double y,
                                const double z)
        {
            const Eigen::Vector3d p000(-x, -y, -z);
            const Eigen::Vector3d p001(-x, -y, z);
            const Eigen::Vector3d p010(-x, y, -z);
            const Eigen::Vector3d p011(-x, y, z);
            const Eigen::Vector3d p100(x, -y, -z);
            const Eigen::Vector3d p101(x, -y, z);
            const Eigen::Vector3d p110(x, y, -z);
            const Eigen::Vector3d p111(x, y, z);
            addEdge(pose, p000, p001);
            addEdge(pose, p000, p010);
            addEdge(pose, p000, p100);
            addEdge(pose, p001, p011);
            addEdge(pose, p001, p101);
            addEdge(pose, p010, p011);
            addEdge(pose, p010, p110);
            addEdge(pose, p011, p111);
            addEdge(pose, p100, p101);
            addEdge(pose, p100, p110);
            addEdge(pose, p101, p111);
            addEdge(pose, p110, p111);
        };

        for (const auto &pose : gatePosesAll())
        {
            addBox(pose, xo, yo, zo);
            addBox(pose, xi, yi, zi);
        }
        return marker;
    }

    visualization_msgs::Marker makeMapBoxMarker() const
    {
        visualization_msgs::Marker marker;
        setMarkerHeader(marker, config_.frameId, "map_box", 0, visualization_msgs::Marker::LINE_LIST);
        setColor(marker, 0.18, 0.18, 0.18, 0.55);
        marker.scale.x = 0.015;

        const double xmin = activeMapBox_(0);
        const double ymin = activeMapBox_(1);
        const double zmin = activeMapBox_(2);
        const double xmax = activeMapBox_(3);
        const double ymax = activeMapBox_(4);
        const double zmax = activeMapBox_(5);

        const auto addEdge = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
        {
            marker.points.push_back(toPoint(a));
            marker.points.push_back(toPoint(b));
        };

        const Eigen::Vector3d p000(xmin, ymin, zmin);
        const Eigen::Vector3d p001(xmin, ymin, zmax);
        const Eigen::Vector3d p010(xmin, ymax, zmin);
        const Eigen::Vector3d p011(xmin, ymax, zmax);
        const Eigen::Vector3d p100(xmax, ymin, zmin);
        const Eigen::Vector3d p101(xmax, ymin, zmax);
        const Eigen::Vector3d p110(xmax, ymax, zmin);
        const Eigen::Vector3d p111(xmax, ymax, zmax);
        addEdge(p000, p001);
        addEdge(p000, p010);
        addEdge(p000, p100);
        addEdge(p001, p011);
        addEdge(p001, p101);
        addEdge(p010, p011);
        addEdge(p010, p110);
        addEdge(p011, p111);
        addEdge(p100, p101);
        addEdge(p100, p110);
        addEdge(p101, p111);
        addEdge(p110, p111);
        return marker;
    }

    void publishStartMarker(const Eigen::Vector3d &start)
    {
        visualization_msgs::Marker start_marker;
        setMarkerHeader(start_marker, config_.frameId, "start_goal", 0, visualization_msgs::Marker::SPHERE);
        start_marker.pose.position = toPoint(start);
        start_marker.scale.x = 0.16;
        start_marker.scale.y = 0.16;
        start_marker.scale.z = 0.16;
        setColor(start_marker, 0.0, 0.65, 0.18, 1.0);
        startGoalMarkerPub_.publish(start_marker);
    }

    void publishStartGoal(const Eigen::Vector3d &start,
                          const Eigen::Vector3d &goal)
    {
        publishStartMarker(start);
        visualization_msgs::Marker goal_marker;
        setMarkerHeader(goal_marker, config_.frameId, "start_goal", 1, visualization_msgs::Marker::SPHERE);
        goal_marker.pose.position = toPoint(goal);
        goal_marker.scale.x = 0.16;
        goal_marker.scale.y = 0.16;
        goal_marker.scale.z = 0.16;
        setColor(goal_marker, 0.1, 0.25, 0.95, 1.0);
        startGoalMarkerPub_.publish(goal_marker);
    }

    double maxDerivNorm(const geometry_utils::Trajectory &traj,
                        const int deriv) const
    {
        if (traj.empty() || traj.getTotalDuration() <= 0.0)
        {
            return 0.0;
        }

        const double total_t = traj.getTotalDuration();
        double max_norm = 0.0;
        for (int i = 0; i <= config_.trajSamples; ++i)
        {
            const double t = total_t * static_cast<double>(i) / static_cast<double>(config_.trajSamples);
            if (deriv == 1)
            {
                max_norm = std::max(max_norm, traj.getVel(t).norm());
            }
            else if (deriv == 2)
            {
                max_norm = std::max(max_norm, traj.getAcc(t).norm());
            }
            else if (deriv == 3)
            {
                max_norm = std::max(max_norm, traj.getJer(t).norm());
            }
            else
            {
                max_norm = std::max(max_norm, traj.getPos(t).norm());
            }
        }
        return max_norm;
    }

    struct FlatnessSample
    {
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d jer = Eigen::Vector3d::Zero();
        double thrust = 0.0;
        Eigen::Vector4d quat = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
        Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    };

    FlatnessSample sampleFlatness(const geometry_utils::Trajectory &traj,
                                 const double raw_t) const
    {
        FlatnessSample sample;
        if (traj.empty())
        {
            return sample;
        }

        const double eval_t = std::clamp(raw_t, 0.0, traj.getTotalDuration());
        sample.pos = traj.getPos(eval_t);
        sample.vel = traj.getVel(eval_t);
        sample.acc = traj.getAcc(eval_t);
        sample.jer = traj.getJer(eval_t);
        const Eigen::Vector3d snap = traj.getSnap(eval_t);

        traj_opt::SE3FlatnessMap flatmap;
        flatmap.setYawMode(false, true);
        traj_opt::SE3FlatnessOutput output;
        if (!flatmap.forward(sample.vel,
                             sample.acc,
                             sample.jer,
                             snap,
                             config_.replayYaw,
                             0.0,
                             config_.gravAcc,
                             output))
        {
            sample.body_rate.setZero();
            sample.thrust = 0.0;
            sample.quat << 1.0, 0.0, 0.0, 0.0;
            sample.rot = Eigen::Matrix3d::Identity();
            return sample;
        }

        sample.thrust = output.thrust;
        sample.body_rate = output.omega;
        sample.rot = output.R;
        Eigen::Quaterniond quat(sample.rot);
        quat.normalize();
        sample.quat << quat.w(), quat.x(), quat.y(), quat.z();
        return sample;
    }

    Eigen::Vector3d bodyEllipsoid() const
    {
        return Eigen::Vector3d(config_.uavCollisionBox.horizontalHalfLen(),
                               config_.uavCollisionBox.horizontalHalfLen(),
                               config_.uavCollisionBox.verticalHalfLen());
    }

    std::vector<Eigen::RowVector4d> gateOpeningFacetRows(const Eigen::Matrix<double, 3, 4> &pose) const
    {
        std::vector<Eigen::RowVector4d> rows;
        const Eigen::Matrix3d rot = pose.leftCols<3>();
        const Eigen::Vector3d gate_center = pose.rightCols<1>();
        const Eigen::Vector3d gate_normal = rot * Eigen::Vector3d::UnitX();
        if (config_.useHolePolygonCorridor && holePolygon_.valid &&
            holePolygon_.vertices.size() >= static_cast<size_t>(config_.holePolygonMinVertices))
        {
            const Eigen::Vector3d polygon_shift = gate_center - holePolygon_.center;
            for (size_t i = 0; i < holePolygon_.vertices.size(); ++i)
            {
                const Eigen::Vector3d a = holePolygon_.vertices[i] + polygon_shift;
                const Eigen::Vector3d b = holePolygon_.vertices[(i + 1) % holePolygon_.vertices.size()] + polygon_shift;
                const Eigen::Vector3d edge = b - a;
                if (edge.norm() < 1.0e-5)
                {
                    continue;
                }
                Eigen::Vector3d inward = gate_normal.cross(edge);
                if (inward.norm() < 1.0e-6)
                {
                    continue;
                }
                inward.normalize();
                if (inward.dot(gate_center - a) < 0.0)
                {
                    inward = -inward;
                }
                rows.push_back(makeHalfspaceRow(-inward, a));
            }
        }

        if (rows.size() < 3)
        {
            const Eigen::Vector3d gate_y = rot * Eigen::Vector3d::UnitY();
            const Eigen::Vector3d gate_z = rot * Eigen::Vector3d::UnitZ();
            const double half_width = 0.5 * config_.gateInnerSize.y();
            const double half_height = 0.5 * config_.gateInnerSize.z();
            rows.clear();
            rows.push_back(makeHalfspaceRow(gate_y, gate_center + half_width * gate_y));
            rows.push_back(makeHalfspaceRow(-gate_y, gate_center - half_width * gate_y));
            rows.push_back(makeHalfspaceRow(gate_z, gate_center + half_height * gate_z));
            rows.push_back(makeHalfspaceRow(-gate_z, gate_center - half_height * gate_z));
        }
        return rows;
    }

    double ellipsoidFacetSignedDistance(const Eigen::RowVector4d &facet,
                                        const Eigen::Vector3d &pos,
                                        const Eigen::Matrix3d &rot,
                                        const Eigen::Vector3d &ellipsoid) const
    {
        const Eigen::Vector3d normal = facet.head<3>().transpose();
        const double support =
            ((rot.transpose() * normal).array() * ellipsoid.array()).matrix().norm();
        return normal.dot(pos) + facet(3) + support + config_.safeMargin;
    }

    struct GateFitReport
    {
        double closest_t = 0.0;
        double closest_normal_offset = std::numeric_limits<double>::infinity();
        double center_opening_signed_distance = -std::numeric_limits<double>::infinity();
        double tunnel_worst_opening_signed_distance = -std::numeric_limits<double>::infinity();
        double tunnel_min_opening_clearance = std::numeric_limits<double>::infinity();
        int tunnel_samples = 0;
    };

    GateFitReport sampleGateFit(const geometry_utils::Trajectory &traj) const
    {
        GateFitReport report;
        if (traj.empty() || traj.getTotalDuration() <= 0.0)
        {
            return report;
        }

        const auto poses = gatePosesAll();
        if (poses.empty())
        {
            return report;
        }

        const Eigen::Vector3d ellipsoid = bodyEllipsoid();
        constexpr int kSamples = 500;
        for (const auto &pose : poses)
        {
            const Eigen::Vector3d center = pose.rightCols<1>();
            const Eigen::Vector3d gate_normal = pose.leftCols<3>() * Eigen::Vector3d::UnitX();
            const std::vector<Eigen::RowVector4d> facets = gateOpeningFacetRows(pose);
            if (facets.empty())
            {
                continue;
            }

            double best_t = 0.0;
            double best_normal_offset = std::numeric_limits<double>::infinity();
            double best_opening_signed = -std::numeric_limits<double>::infinity();
            for (int i = 0; i <= kSamples; ++i)
            {
                const double t = traj.getTotalDuration() *
                                 static_cast<double>(i) / static_cast<double>(kSamples);
                const FlatnessSample sample = sampleFlatness(traj, t);
                const double normal_offset = std::abs(gate_normal.dot(sample.pos - center));
                if (normal_offset >= best_normal_offset)
                {
                    continue;
                }

                best_t = t;
                best_normal_offset = normal_offset;
                double max_opening_signed = -std::numeric_limits<double>::infinity();
                for (const auto &facet : facets)
                {
                    max_opening_signed =
                        std::max(max_opening_signed,
                                 ellipsoidFacetSignedDistance(facet, sample.pos, sample.rot, ellipsoid));
                }
                best_opening_signed = max_opening_signed;
            }

            if (best_normal_offset < report.closest_normal_offset)
            {
                report.closest_normal_offset = best_normal_offset;
                report.closest_t = best_t;
            }
            report.center_opening_signed_distance =
                std::max(report.center_opening_signed_distance, best_opening_signed);
            report.tunnel_worst_opening_signed_distance =
                std::max(report.tunnel_worst_opening_signed_distance, best_opening_signed);
            report.tunnel_min_opening_clearance =
                std::min(report.tunnel_min_opening_clearance, -best_opening_signed);
            report.tunnel_samples++;
        }
        return report;
    }

    void reportGateFit(const geometry_utils::Trajectory &traj) const
    {
        const GateFitReport gate_fit = sampleGateFit(traj);
        ROS_INFO_STREAM("gate ellipsoid fit sample:"
                        << " closest_t=" << gate_fit.closest_t
                        << " normal_offset=" << gate_fit.closest_normal_offset
                        << " center_opening_signed_dist=" << gate_fit.center_opening_signed_distance
                        << " tunnel_worst_opening_signed_dist=" << gate_fit.tunnel_worst_opening_signed_distance
                        << " tunnel_min_opening_clearance=" << gate_fit.tunnel_min_opening_clearance
                        << " tunnel_samples=" << gate_fit.tunnel_samples);
        if (gate_fit.center_opening_signed_distance > 0.0 ||
            gate_fit.tunnel_worst_opening_signed_distance > 0.0)
        {
            ROS_WARN_STREAM("Positive gate opening signed distance means the planned ellipsoid plus safe margin "
                            << "still violates at least one gate opening face in the sampled check.");
        }
    }

    void publishTrajectory(const geometry_utils::Trajectory &traj)
    {
        visualization_msgs::Marker marker;
        setMarkerHeader(marker, config_.frameId, "trajectory", 0, visualization_msgs::Marker::LINE_STRIP);
        setColor(marker, 0.0, 0.20, 1.0, 1.0);
        marker.scale.x = config_.trajVizWidth;

        nav_msgs::Path path;
        path.header.frame_id = config_.frameId;
        path.header.stamp = ros::Time::now();

        const double total_t = traj.getTotalDuration() + std::max(0.0, config_.appendedDuration);
        for (int i = 0; i <= config_.trajSamples; ++i)
        {
            const double t = total_t * static_cast<double>(i) / static_cast<double>(config_.trajSamples);
            const double eval_t = std::min(t, traj.getTotalDuration());
            const FlatnessSample sample = sampleFlatness(traj, eval_t);
            marker.points.push_back(toPoint(sample.pos));

            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position = toPoint(sample.pos);
            pose.pose.orientation = toQuaternionMsg(sample.quat);
            path.poses.push_back(pose);
        }

        trajMarkerPub_.publish(marker);
        trajPathPub_.publish(path);
    }

    void publishVehicleTrajectoryMarkers(const geometry_utils::Trajectory &traj)
    {
        visualization_msgs::MarkerArray arr;
        if (traj.empty() || traj.getTotalDuration() <= 0.0)
        {
            return;
        }

        const Eigen::Vector3d ellipsoid = bodyEllipsoid();
        const int samples = std::max(config_.vehicleEllipsoidSamples, 2);
        arr.markers.reserve(samples + 1);

        visualization_msgs::Marker delete_previous;
        setMarkerHeader(delete_previous, config_.frameId,
                        "vehicle_ellipsoids", 0, visualization_msgs::Marker::SPHERE);
        delete_previous.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(delete_previous);

        for (int i = 0; i < samples; ++i)
        {
            const double t = traj.getTotalDuration() * static_cast<double>(i) /
                             static_cast<double>(samples - 1);
            const FlatnessSample sample = sampleFlatness(traj, t);

            visualization_msgs::Marker marker;
            setMarkerHeader(marker, config_.frameId,
                            "vehicle_ellipsoids", i, visualization_msgs::Marker::SPHERE);
            marker.pose.position = toPoint(sample.pos);
            marker.pose.orientation = toQuaternionMsg(sample.quat);
            marker.scale.x = 2.0 * ellipsoid.x();
            marker.scale.y = 2.0 * ellipsoid.y();
            marker.scale.z = 2.0 * ellipsoid.z();
            setColor(marker, 0.02, 0.02, 0.02, config_.vehicleEllipsoidAlpha);
            arr.markers.push_back(marker);
        }

        vehicleEllipsoidsPub_.publish(arr);
    }

    void publishCommandsAt(const double raw_t,
                           const ros::Time &stamp)
    {
        if (!config_.publishCommands ||
            !hasTraj_ ||
            lastTraj_.empty())
        {
            return;
        }

        const double spline_t = lastTraj_.getTotalDuration();
        const double total_t = spline_t + std::max(0.0, config_.appendedDuration);
        if (spline_t <= 0.0 || total_t <= 0.0)
        {
            return;
        }

        double command_t = std::max(0.0, raw_t);
        if (config_.loopReplay)
        {
            command_t = std::fmod(command_t, total_t);
        }
        else
        {
            command_t = std::min(command_t, total_t);
        }

        const double eval_t = std::min(command_t, spline_t);
        const uint8_t trajectory_flag = command_t >= spline_t
                                            ? quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED
                                            : quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

        const FlatnessSample sample = sampleFlatness(lastTraj_, eval_t);

        Eigen::Quaterniond quat(sample.quat(0), sample.quat(1), sample.quat(2), sample.quat(3));
        quat.normalize();
        const Eigen::Vector3d force = quat * Eigen::Vector3d::UnitZ() * sample.thrust;

        quadrotor_msgs::PositionCommand pos_cmd;
        pos_cmd.header.stamp = stamp;
        pos_cmd.header.frame_id = config_.frameId;
        pos_cmd.position = toPoint(sample.pos);
        pos_cmd.velocity = toVector3(sample.vel);
        pos_cmd.acceleration = toVector3(sample.acc);
        pos_cmd.yaw = config_.replayYaw;
        pos_cmd.yaw_dot = 0.0;
        pos_cmd.trajectory_id = trajId_;
        pos_cmd.trajectory_flag = trajectory_flag;
        for (int i = 0; i < 3; ++i)
        {
            pos_cmd.kx[i] = 0.0;
            pos_cmd.kv[i] = 0.0;
        }

        quadrotor_msgs::SO3Command so3_cmd;
        so3_cmd.header = pos_cmd.header;
        so3_cmd.force = toVector3(force);
        so3_cmd.orientation.x = quat.x();
        so3_cmd.orientation.y = quat.y();
        so3_cmd.orientation.z = quat.z();
        so3_cmd.orientation.w = quat.w();
        for (int i = 0; i < 3; ++i)
        {
            so3_cmd.kR[i] = 0.0;
            so3_cmd.kOm[i] = 0.0;
        }
        so3_cmd.aux.current_yaw = config_.replayYaw;
        so3_cmd.aux.kf_correction = 0.0;
        so3_cmd.aux.angle_corrections[0] = 0.0;
        so3_cmd.aux.angle_corrections[1] = 0.0;
        so3_cmd.aux.enable_motors = true;
        so3_cmd.aux.use_external_yaw = false;

        positionCommandPub_.publish(pos_cmd);
        so3CommandPub_.publish(so3_cmd);
    }

    void publishVehicleMarker()
    {
        if (!hasTraj_ || lastTraj_.empty())
        {
            return;
        }

        const double spline_t = lastTraj_.getTotalDuration();
        const double total_t = spline_t + std::max(0.0, config_.appendedDuration);
        if (spline_t <= 0.0 || total_t <= 0.0)
        {
            return;
        }

        double t = ros::Time::now().toSec() - trajStamp_;
        if (config_.loopReplay)
        {
            t = std::fmod(std::max(0.0, t), total_t);
        }
        else
        {
            t = std::min(std::max(0.0, t), total_t);
        }

        const double eval_t = std::min(t, spline_t);
        const FlatnessSample sample = sampleFlatness(lastTraj_, eval_t);
        const Eigen::Vector3d ellipsoid = bodyEllipsoid();

        visualization_msgs::Marker marker;
        setMarkerHeader(marker, config_.frameId, "vehicle_ellipsoid", 0, visualization_msgs::Marker::SPHERE);
        marker.pose.position = toPoint(sample.pos);
        marker.pose.orientation = toQuaternionMsg(sample.quat);
        marker.scale.x = 2.0 * ellipsoid.x();
        marker.scale.y = 2.0 * ellipsoid.y();
        marker.scale.z = 2.0 * ellipsoid.z();
        setColor(marker, 0.05, 0.05, 0.05, 0.95);
        vehicleMarkerPub_.publish(marker);
    }

    void timerCallback(const ros::TimerEvent &)
    {
        publishStaticMarkers();
        publishVehicleMarker();
        maybeAutoPlanMirror("timer");
    }

    void commandTimerCallback(const ros::TimerEvent &event)
    {
        publishCommandsAt(event.current_real.toSec() - trajStamp_, event.current_real);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "se3_gcopter_gate_planning_node");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    GatePlanner planner(GatePlannerConfig(private_node), node);
    ros::spin();
    return 0;
}
