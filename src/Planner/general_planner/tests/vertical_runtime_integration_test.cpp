#define LOG_FILE_DIR(name) (std::string("/tmp/general_planner_vertical_test/") + (name))
#include <general_core/planner_runtime/planner_supervisor.hpp>
#include <ros_interface/ros1/fsm_ros1.hpp>
#include <cstdlib>
#include <iostream>
#include <limits>

// Real supervisor, FSM, map queries, command sampler and gateway, driven by
// an ideal plant on an isolated ROS master. No vehicle master is contacted.
int main(int argc, char **argv) {
    const char *master = std::getenv("ROS_MASTER_URI");
    if (!master || std::string(master) != "http://127.0.0.1:11386") {
        std::cerr << "requires isolated ROS_MASTER_URI=http://127.0.0.1:11386\n";
        return 2;
    }
    ros::init(argc, argv, "vertical_runtime_integration_test");
    ros::NodeHandle nh("~");
    nh.setParam("initial_mode", "hold");
    nh.setParam("serial_handover", false);
    nh.setParam("odometry_topic", "/vertical_test/odom");
    nh.setParam("hover_hold_duration", 0.15);
    auto grid = std::make_shared<rog_map::ROGMapROS>(nh, std::string(ROOT_DIR) + "tests/gate_unknown_map.yaml");
    auto map = std::make_shared<general_planner::MapManager>(grid);
    map->enableIndependentOdometry();
    // Observe a horizontal ray only, leaving the ascent/descent column unknown
    // as it can be with a lidar whose vertical field of view is limited.
    rog_map::PointCloud cloud;
    rog_map::PointType hit;
    hit.x = 2.925; hit.y = .075; hit.z = 1.275; hit.intensity = 1;
    cloud.push_back(hit);
    for (int i = 0; i < 8; ++i)
        map->updateMap(cloud, {Eigen::Vector3d(.075, .075, 1.275), Eigen::Quaterniond::Identity()});
    for (double z : {1.475, 1.875}) {
        if (map->getGridType(Eigen::Vector3d(.075, .075, z)) != rog_map::UNKNOWN) {
            std::cerr << "FAIL: vertical test targets must start in unknown space\n";
            return 1;
        }
    }

    general_planner::planner_runtime::PlannerCommandGateway gateway(nh);
    general_planner::planner_runtime::PlannerSupervisor supervisor(nh, gateway, [] {
        general_planner::planner_runtime::GlobalMapStatus status;
        status.odom_valid = status.map_ready = status.topology_ready = true;
        return status;
    });
    fsm::FsmRos1 navigation;
    navigation.init(nh, std::string(ROOT_DIR) + "config/task_planner_runtime_state2state.yaml",
                    map, "/planning/navigation/pos_cmd");
    general_planner::PlannerStatus latest;
    general_planner::NavigationGoalRequest request;
    general_planner::NavigationGoalAck ack;
    auto status_sub = nh.subscribe<general_planner::PlannerStatus>("/planner/status", 10,
        [&](const general_planner::PlannerStatusConstPtr &msg) { latest = *msg; });
    auto request_sub = nh.subscribe<general_planner::NavigationGoalRequest>("/planning/navigation/goal_request", 10,
        [&](const general_planner::NavigationGoalRequestConstPtr &msg) { request = *msg; });
    auto ack_sub = nh.subscribe<general_planner::NavigationGoalAck>("/planning/navigation/goal_ack", 10,
        [&](const general_planner::NavigationGoalAckConstPtr &msg) { ack = *msg; });
    nav_msgs::Odometry odom;
    odom.header.frame_id = "world";
    odom.pose.pose.position.x = odom.pose.pose.position.y = .075;
    odom.pose.pose.position.z = 1.275;
    constexpr double yaw0 = .4;
    odom.pose.pose.orientation.w = std::cos(yaw0 / 2);
    odom.pose.pose.orientation.z = std::sin(yaw0 / 2);
    bool command_valid = true;
    double z_min = 1e9, z_max = -1e9;
    auto command_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 1,
        [&](const quadrotor_msgs::PositionCommandConstPtr &msg) {
            if (std::abs(msg->position.x - .075) > 1e-6 || std::abs(msg->position.y - .075) > 1e-6 ||
                std::abs(msg->yaw - yaw0) > 1e-6 || std::abs(msg->velocity.z) > .50001 ||
                std::abs(msg->acceleration.z) > .50001 || std::abs(msg->jerk.z) > 1.00001)
                command_valid = false;
            z_min = std::min(z_min, msg->position.z);
            z_max = std::max(z_max, msg->position.z);
            odom.pose.pose.position = msg->position;
            odom.twist.twist.linear = msg->velocity;
        });
    auto odom_pub = nh.advertise<nav_msgs::Odometry>("/vertical_test/odom", 1);
    auto mode_pub = nh.advertise<general_planner::PlannerModeRequest>("/planner/mode_request", 1);
    auto text_pub = nh.advertise<std_msgs::String>("/planner/mode_request_text", 1);
    auto height_pub = nh.advertise<std_msgs::Float64>("/planner/vertical/height", 1);
    auto goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_3d", 1);
    auto wait = [&](const std::function<bool()> &predicate, double seconds) {
        const auto begin = ros::WallTime::now();
        while (ros::ok() && (ros::WallTime::now() - begin).toSec() < seconds) {
            odom.header.stamp = ros::Time::now();
            odom_pub.publish(odom);
            rog_map::RobotState state{};
            state.rcv = true; state.rcv_time = odom.header.stamp.toSec();
            state.p << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
            state.v << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z;
            state.a.setZero(); state.yaw = yaw0;
            state.q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw0, Eigen::Vector3d::UnitZ()));
            map->updateOdometrySnapshot(state);
            ros::spinOnce();
            if (predicate()) return true;
            ros::WallDuration(.005).sleep();
        }
        return false;
    };
    const auto check = [&](bool ok, const char *why) {
        if (!ok) std::cerr << "FAIL: " << why << " status=" << latest.reason << " ack=" << ack.reason << '\n';
        return ok;
    };
    const auto height = [&](double delta) { std_msgs::Float64 msg; msg.data = delta; height_pub.publish(msg); };
    if (!check(wait([&] { return latest.ready_for_new_task && height_pub.getNumSubscribers() && mode_pub.getNumSubscribers(); }, 5), "boot")) return 1;
    height(.6); wait([] { return false; }, .2);
    if (!check(request.request_id == 0, "height accepted outside vertical")) return 1;
    general_planner::PlannerModeRequest mode;
    mode.mode = mode.MODE_VERTICAL; mode.request_id = 1; mode_pub.publish(mode);
    if (!check(wait([&] { return latest.active_mode_str == "vertical" && latest.ready_for_new_task; }, 5), "typed vertical activation")) return 1;
    for (double invalid : {0.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        height(invalid); wait([] { return false; }, .1);
    }
    geometry_msgs::PoseStamped goal;
    goal.pose.position.z = 2; goal.pose.orientation.w = 1; goal_pub.publish(goal);
    wait([] { return false; }, .2);
    if (!check(request.request_id == 0, "invalid height or 3D goal started vertical")) return 1;
    height(.6);
    if (!check(wait([&] { return latest.phase_str == "executing"; }, 3), "upward execution")) return 1;
    const auto first_request = request.request_id;
    if (!check(std::abs(request.goal.pose.position.z - 1.875) < 1e-6, "height was not relative")) return 1;
    height(.2); height(-.2); wait([] { return false; }, .2);
    if (!check(request.request_id == first_request, "busy height replaced trajectory")) return 1;
    if (!check(wait([&] { return latest.task_result_str == "succeeded" && latest.ready_for_new_task; }, 9), "upward completion")) return 1;
    if (!check(command_valid && std::abs(odom.pose.pose.position.z - 1.875) < .01 && z_max <= 1.875001,
               "upward geometry/dynamics")) return 1;
    height(-.4);
    if (!check(wait([&] { return request.request_id > first_request && latest.phase_str == "executing"; }, 3), "downward execution")) return 1;
    if (!check(wait([&] { return latest.task_result_str == "succeeded" && latest.ready_for_new_task; }, 9), "downward completion")) return 1;
    if (!check(command_valid && std::abs(odom.pose.pose.position.z - 1.475) < .01 && z_min >= 1.274999,
               "downward geometry/dynamics")) return 1;
    // A ceiling-crossing request is rejected without moving or projecting xy.
    auto previous = request.request_id;
    height(3.0);
    if (!check(wait([&] { return ack.request_id > previous && ack.result == ack.REJECTED; }, 2), "ceiling rejection")) return 1;
    // Inject a newly seen obstacle after accepting a trajectory through unknown space.
    height(.5);
    if (!check(wait([&] { return latest.phase_str == "executing"; }, 3), "obstacle scenario start")) return 1;
    cloud[0].x = .075; cloud[0].z = 1.975;
    grid->updateOccPointCloud(cloud);
    if (!check(wait([&] { return latest.task_result_str == "failed"; }, 3), "new obstacle did not abort")) return 1;
    std_msgs::String hold; hold.data = "hold"; text_pub.publish(hold);
    if (!check(wait([&] { return latest.active_mode_str == "hold" && latest.ready_for_new_task; }, 4), "hold cancellation")) return 1;
    std::cout << "vertical_runtime_integration_test passed: unknown-space ascent/descent, signed height, mode gate, limits, fixed xy/yaw, busy rejection, completion, ceiling and live collision\n";
}
