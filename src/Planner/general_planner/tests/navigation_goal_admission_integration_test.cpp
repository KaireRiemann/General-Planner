#include <general_core/planner_runtime/planner_supervisor.hpp>
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv) {
  const char *master = std::getenv("ROS_MASTER_URI");
  if (!master || std::string(master) != "http://127.0.0.1:11384") {
    std::cerr << "requires isolated ROS_MASTER_URI=http://127.0.0.1:11384\n";
    return 2;
  }
  ros::init(argc, argv, "navigation_goal_admission_integration_test");
  ros::NodeHandle nh("~");
  nh.setParam("initial_mode", "state2state");
  nh.setParam("serial_handover", false);
  nh.setParam("odometry_topic", "/admission_test/odom");
  nh.setParam("navigation_goal_ack_timeout", 0.5);
  general_planner::planner_runtime::PlannerCommandGateway gateway(nh);
  general_planner::planner_runtime::PlannerSupervisor supervisor(nh, gateway, [] {
    general_planner::planner_runtime::GlobalMapStatus s;
    s.odom_valid = s.map_ready = s.topology_ready = true;
    return s;
  });
  general_planner::PlannerStatus latest;
  general_planner::NavigationGoalRequest request;
  auto status_sub = nh.subscribe<general_planner::PlannerStatus>("/planner/status", 20,
      [&](const general_planner::PlannerStatusConstPtr &m) { latest = *m; });
  auto request_sub = nh.subscribe<general_planner::NavigationGoalRequest>("/planning/navigation/goal_request", 20,
      [&](const general_planner::NavigationGoalRequestConstPtr &m) { request = *m; });
  auto odom_pub = nh.advertise<nav_msgs::Odometry>("/admission_test/odom", 1);
  auto goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_3d", 1);
  auto ack_pub = nh.advertise<general_planner::NavigationGoalAck>("/planning/navigation/goal_ack", 10);
  auto nav_pub = nh.advertise<std_msgs::String>("/planning/navigation/status", 10);
  auto cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/navigation/pos_cmd", 1);
  bool moving_with_zero_twist = true;
  double x = 0;
  std::string navigation_state;
  auto wait = [&](const std::function<bool()> &predicate, double seconds) {
    const auto start = ros::WallTime::now();
    while (ros::ok() && (ros::WallTime::now()-start).toSec() < seconds) {
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now(); odom.header.frame_id = "world";
      if (moving_with_zero_twist) x += .01;
      odom.pose.pose.position.x = x; odom.pose.pose.position.z = 1;
      odom.pose.pose.orientation.w = 1;
      odom_pub.publish(odom);
      quadrotor_msgs::PositionCommand command;
      command.position = odom.pose.pose.position;
      command.trajectory_flag = command.TRAJECTORY_STATUS_READY;
      cmd_pub.publish(command);
      if (!navigation_state.empty()) {
        std_msgs::String state; state.data = navigation_state;
        nav_pub.publish(state);
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
  wait([] { return false; }, 1.5);
  if (!check(latest.active_mode_str != "state2state", "moving zero-twist pose passed hover gate")) return 1;
  moving_with_zero_twist = false;
  if (!check(wait([&] { return latest.active_mode_str == "state2state" && latest.ready_for_new_task &&
      goal_pub.getNumSubscribers() && ack_pub.getNumSubscribers(); }, 5), "boot")) return 1;
  const auto epoch = latest.task_epoch;
  const std::string prefix = " " + std::to_string(epoch) + " 1 ";
  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = 10; goal.pose.position.z = 1; goal.pose.orientation.w = 1;
  goal_pub.publish(goal);
  if (!check(wait([&] { return request.request_id > 0; }, 1), "first request")) return 1;
  general_planner::NavigationGoalAck ack;
  ack.task_epoch = epoch; ack.request_id = request.request_id;
  ack.goal_sequence = 1; ack.result = ack.ACCEPTED;
  ack_pub.publish(ack);
  navigation_state = "FOLLOW_TRAJ" + prefix + "ACTIVE QUIESCENT";
  if (!check(wait([&] { return latest.phase_str == "executing"; }, 1), "execution")) return 1;
  for (const auto *reason : {"too_close", "non_finite_position", "invalid_quaternion", "occupied_or_no_free_projection"}) {
    const auto previous = request.request_id;
    goal_pub.publish(goal);
    if (!check(wait([&] { return request.request_id > previous; }, 1), "replacement request")) return 1;
    ack.request_id = request.request_id; ack.result = ack.REJECTED; ack.reason = reason;
    ack_pub.publish(ack);
    wait([] { return false; }, .15);
    if (!check(latest.command_owner == 1 && latest.task_result_str != "failed", "rejection retired active goal")) return 1;
  }
  // Same goal is an idempotent acknowledgement of the active sequence.
  auto previous = request.request_id;
  goal_pub.publish(goal);
  if (!check(wait([&] { return request.request_id > previous; }, 1), "duplicate request")) return 1;
  ack.request_id = request.request_id; ack.result = ack.ALREADY_ACTIVE; ack.reason = "duplicate";
  ack_pub.publish(ack); wait([] { return false; }, .1);
  // An old ACK must not clear a newer pending request.
  previous = request.request_id; goal_pub.publish(goal);
  if (!check(wait([&] { return request.request_id > previous; }, 1), "new request")) return 1;
  ack_pub.publish(ack);
  wait([] { return false; }, .1);
  navigation_state = "WAIT_GOAL" + prefix + "IDLE QUIESCENT";
  wait([] { return false; }, .1);
  if (!check(latest.task_result_str != "succeeded" && !latest.ready_for_new_task,
             "old completion retired a pending replacement before ACK")) return 1;
  ack.request_id = request.request_id; ack.result = ack.REJECTED; ack.reason = "too_close";
  ack_pub.publish(ack);
  if (!check(wait([&] { return latest.task_result_str == "succeeded"; }, 1), "accepted goal did not complete after rejected replacements")) return 1;
  navigation_state.clear();
  if (!check(wait([&] { return latest.ready_for_new_task && latest.task_epoch > epoch; }, 3), "completion recovery")) return 1;
  previous = request.request_id; goal_pub.publish(goal);
  if (!check(wait([&] { return request.request_id > previous; }, 1), "timeout request")) return 1;
  if (!check(wait([&] { return latest.task_result_str == "failed" &&
      latest.reason.find("admission timeout") != std::string::npos; }, 2), "missing ACK not bounded")) return 1;
  std::cout << "navigation admission, replacement rejection, duplicate/stale ACK, completion, hover and ACK deadline: PASS\n";
}
