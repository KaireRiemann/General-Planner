#include <general_core/planner_runtime/planner_supervisor.hpp>
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv) {
  const char *master = std::getenv("ROS_MASTER_URI");
  if (!master || std::string(master) != "http://127.0.0.1:11382") {
    std::cerr << "requires isolated ROS_MASTER_URI=http://127.0.0.1:11382\n";
    return 2;
  }
  const bool deadline_test = argc > 1 && std::string(argv[1]) == "deadline";
  ros::init(argc, argv, "target_route_handshake_integration_test");
  ros::NodeHandle nh("~");
  nh.setParam("initial_mode", "target_exploration");
  nh.setParam("serial_handover", false);
  nh.setParam("exploration_first_command_timeout", 4.0);
  nh.setParam("source_startup_grace_duration", 0.5);
  nh.setParam("odometry_topic", "/route_handshake/odom");
  general_planner::planner_runtime::PlannerCommandGateway gateway(nh);
  general_planner::planner_runtime::PlannerSupervisor supervisor(nh, gateway, [] {
    general_planner::planner_runtime::GlobalMapStatus state;
    state.odom_valid = state.map_ready = true;
    state.topology_ready = false;  // no graph must not block target admission
    return state;
  });
  general_planner::PlannerStatus latest;
  auto status_sub = nh.subscribe<general_planner::PlannerStatus>("/planner/status", 20,
      [&](const general_planner::PlannerStatusConstPtr &m) { latest = *m; });
  auto odom_pub = nh.advertise<nav_msgs::Odometry>("/route_handshake/odom", 1);
  auto goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/planner/exploration/trigger", 1);
  auto task_pub = nh.advertise<std_msgs::String>("/planning/exploration/status", 10);
  auto command_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/exploration/pos_cmd", 1);
  bool send_commands = false;
  auto wait = [&](const std::function<bool()> &predicate, double seconds) {
    const auto began = ros::WallTime::now();
    while (ros::ok() && (ros::WallTime::now() - began).toSec() < seconds) {
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now(); odom.header.frame_id = "world";
      odom.pose.pose.position.z = 1.5; odom.pose.pose.orientation.w = 1;
      odom_pub.publish(odom);
      if (send_commands) {
        quadrotor_msgs::PositionCommand command;
        command.header.stamp = ros::Time::now(); command.position.z = 1.5;
        command_pub.publish(command);
      }
      ros::spinOnce();
      if (predicate()) return true;
      ros::WallDuration(.01).sleep();
    }
    return false;
  };
  auto check = [&](bool ok, const char *reason) {
    if (!ok) std::cerr << reason << ": " << latest.reason << '\n';
    return ok;
  };
  if (!check(wait([&] { return latest.active_mode_str == "target_exploration" &&
      latest.ready_for_new_task && goal_pub.getNumSubscribers() && task_pub.getNumSubscribers(); }, 6), "boot")) return 1;
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world"; goal.pose.position.x = 10;
  goal.pose.position.z = 1.5; goal.pose.orientation.w = 1;
  goal_pub.publish(goal);
  if (!check(wait([&] { return !latest.ready_for_new_task && !latest.task_id.empty(); }, 1), "goal admission")) return 1;
  const std::string task_id = latest.task_id;
  if (!deadline_test) {
    std_msgs::String report; report.data = "WAITING_LOCAL_PLAN " + task_id;
    task_pub.publish(report);
    wait([] { return false; }, 2.0);  // exceeds ordinary source startup grace
    if (!check(latest.phase_str == "waiting_input" && latest.command_owner == 0 &&
        !latest.ready_for_new_task && latest.task_result_str != "failed", "premature source authorization")) return 1;
    report.data = "RUNNING " + task_id;
    send_commands = true;
    task_pub.publish(report);
    if (!check(wait([&] { return latest.command_owner == 2; }, 1), "first command handoff")) return 1;
    wait([] { return false; }, 2.5);  // original first-command deadline is retired
    if (!check(latest.task_result_str != "failed" && latest.command_owner == 2,
               "deadline not retired")) return 1;
  } else {
    // No status callback at all: deadline is armed at START and retries cannot
    // extend it. This models a blocked world/planning queue.
    if (!check(wait([&] { return latest.task_result_str == "failed"; }, 5), "independent deadline")) return 1;
    std_msgs::String report; report.data = "RUNNING " + task_id;
    send_commands = true; task_pub.publish(report);
    wait([] { return false; }, .25);
    if (!check(latest.command_owner == 0, "late RUNNING revived timed-out source")) return 1;
    report.data = "PAUSED " + task_id; task_pub.publish(report);
    wait([] { return false; }, 1.0);
    report.data = "RUNNING " + task_id; task_pub.publish(report);
    wait([] { return false; }, .25);
    if (!check(latest.command_owner == 0, "stale task revived after hold")) return 1;
  }
  std::cout << "target route " << (deadline_test ? "independent first-command deadline" : "WAITING_LOCAL_PLAN -> first command") << ": PASS\n";
  return 0;
}
