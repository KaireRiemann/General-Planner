#include <general_core/planner_runtime/planner_supervisor.hpp>
#include <cstdlib>
#include <cmath>
#include <iostream>

int main(int argc, char **argv) {
  // This fixture publishes synthetic vehicle data. Never allow the normal
  // vehicle master: run only with the explicitly isolated test master.
  const char *master = std::getenv("ROS_MASTER_URI");
  if (!master || (std::string(master) != "http://127.0.0.1:11381" &&
                  std::string(master) != "http://127.0.0.1:11383")) {
    std::cerr << "requires isolated ROS_MASTER_URI on loopback port 11381 or 11383\n";
    return 2;
  }
  const bool tracking_test = argc > 1 && std::string(argv[1]) == "tracking";
  const bool deadline_test = argc > 1 && std::string(argv[1]) == "deadline";
  ros::init(argc, argv, "planning_failure_integration_test");
  ros::NodeHandle nh("~");
  nh.setParam("initial_mode", "state2state");
  nh.setParam("serial_handover", false);
  nh.setParam("navigation_planning_timeout", 0.5);
  nh.setParam("odometry_topic", "/failure_test/odom");
  general_planner::planner_runtime::PlannerCommandGateway gateway(nh);
  general_planner::planner_runtime::PlannerSupervisor supervisor(nh, gateway, [] {
    general_planner::planner_runtime::GlobalMapStatus s;
    s.odom_valid = s.map_ready = s.topology_ready = true;
    s.map_revision = s.topo_revision = 1;
    return s;
  });
  general_planner::PlannerStatus latest;
  auto status = nh.subscribe<general_planner::PlannerStatus>("/planner/status", 10,
      [&](const general_planner::PlannerStatusConstPtr &m) { latest = *m; });
  auto odom_pub = nh.advertise<nav_msgs::Odometry>("/failure_test/odom", 1);
  auto goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_3d", 1);
  std::string last_arm;
  auto commands = nh.subscribe<std_msgs::String>("/planning/navigation/command", 10,
      [&](const std_msgs::StringConstPtr &m) {
        if (m->data.rfind("ARM ", 0) == 0) last_arm = m->data;
      });
  auto mode_pub = nh.advertise<std_msgs::String>("/planner/mode_request_text", 1);
  auto nav_pub = nh.advertise<std_msgs::String>("/planning/navigation/status", 1);
  auto cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/navigation/pos_cmd", 1);
  quadrotor_msgs::PositionCommand endpoint;
  endpoint.position.x = 17.5; endpoint.position.y = 2.; endpoint.position.z = 3.;
  endpoint.yaw = 1.1;
  bool send_endpoint = false;
  bool check_hold_endpoint = false, bad_hold_endpoint = false, saw_hold_endpoint = false;
  auto output = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50,
      [&](const quadrotor_msgs::PositionCommandConstPtr &m) {
        if (!check_hold_endpoint) return;
        saw_hold_endpoint = true;
        if (std::abs(m->position.x - endpoint.position.x) > 1.e-6 ||
            std::abs(m->position.y - endpoint.position.y) > 1.e-6 ||
            std::abs(m->position.z - endpoint.position.z) > 1.e-6 ||
            std::abs(m->yaw - endpoint.yaw) > 1.e-6) bad_hold_endpoint = true;
      });
  auto wait = [&](const std::function<bool()> &predicate, double seconds) {
    const auto start = ros::WallTime::now();
    while (ros::ok() && (ros::WallTime::now() - start).toSec() < seconds) {
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now(); odom.header.frame_id = "world";
      odom.pose.pose.orientation.w = 1; odom.pose.pose.position.z = 1;
      odom_pub.publish(odom);
      if (send_endpoint) cmd_pub.publish(endpoint);
      ros::spinOnce();
      if (predicate()) return true;
      ros::WallDuration(.01).sleep();
    }
    return false;
  };
  if (!wait([&] { return latest.active_mode_str == "state2state" &&
                        latest.ready_for_new_task && goal_pub.getNumSubscribers(); }, 5)) {
    std::cerr << "boot failed: " << latest.reason << '\n'; return 1;
  }
  if (tracking_test) {
    std_msgs::String request;
    request.data = "tracking";
    if (!wait([&] { return mode_pub.getNumSubscribers() > 0; }, 2)) return 1;
    mode_pub.publish(request);
    if (!wait([&] { return latest.active_mode_str == "tracking" &&
                          latest.phase_str == "waiting_input" &&
                          last_arm.find(" tracking") != std::string::npos; }, 5)) {
      std::cerr << "tracking activation failed: " << latest.reason << '\n'; return 1;
    }
    const auto tracking_epoch = latest.task_epoch;
    send_endpoint = true;
    std_msgs::String nav;
    nav.data = "GENERATE_TRAJ " + std::to_string(tracking_epoch) + " 0 ACTIVE";
    nav_pub.publish(nav);
    if (!wait([&] { return latest.phase_str == "planning" && latest.command_owner == 1; }, .3)) return 1;
    nav.data = "FOLLOW_TRAJ " + std::to_string(tracking_epoch) + " 0 ACTIVE";
    nav_pub.publish(nav);
    if (!wait([&] { return latest.phase_str == "executing"; }, .3)) return 1;
    nav.data = "TRACKING_BRAKING " + std::to_string(tracking_epoch) + " 0 ACTIVE";
    nav_pub.publish(nav);
    if (!wait([&] { return latest.phase_str == "braking" && latest.command_owner == 1; }, .5)) return 1;
    check_hold_endpoint = true;
    nav.data = "TRACKING_LOST " + std::to_string(tracking_epoch) + " 0 IDLE QUIESCENT";
    nav_pub.publish(nav);
    if (!wait([&] { return latest.phase_str == "waiting_input" &&
                          latest.active_mode_str == "tracking" &&
                          latest.task_epoch == tracking_epoch &&
                          latest.reason.find("lost") != std::string::npos; }, .5)) return 1;
    nav.data = "GENERATE_TRAJ " + std::to_string(tracking_epoch) + " 0 ACTIVE";
    nav_pub.publish(nav);
    if (!wait([&] { return latest.phase_str == "planning" && latest.command_owner == 1; }, .5)) return 1;
    nav.data = "FOLLOW_TRAJ " + std::to_string(tracking_epoch) + " 0 ACTIVE";
    nav_pub.publish(nav);
    if (!wait([&] { return latest.phase_str == "executing"; }, .5)) return 1;
    nav.data = "WAIT_GOAL " + std::to_string(tracking_epoch) + " 0 IDLE QUIESCENT";
    nav_pub.publish(nav);
    if (!wait([&] { return latest.phase_str == "waiting_input" &&
                          latest.task_result_str == "none" && latest.command_owner == 0; }, .3)) return 1;
    wait([] { return false; }, .15);
    if (!saw_hold_endpoint || bad_hold_endpoint) {
      std::cerr << "tracking loss/reacquisition output returned to old hold/odom pose\n";
      return 1;
    }
    check_hold_endpoint = false;
    send_endpoint = false;
    request.data = "state2state";
    mode_pub.publish(request);
    if (!wait([&] { return latest.active_mode_str == "state2state" &&
                          last_arm.find(" state2state") != std::string::npos; }, 5)) return 1;
    // A late tracking state must not reopen the previous command epoch.
    nav.data = "FOLLOW_TRAJ " + std::to_string(tracking_epoch) + " 0 ACTIVE";
    nav_pub.publish(nav);
    wait([] { return false; }, .15);
    if (latest.phase_str == "executing") return 1;
    request.data = "tracking";
    mode_pub.publish(request);
    if (!wait([&] { return latest.active_mode_str == "tracking"; }, 5)) return 1;
    request.data = "hold";
    mode_pub.publish(request);
    if (!wait([&] { return latest.active_mode_str == "hold" && latest.command_owner == 0; }, 5)) return 1;
    std::cout << "tracking supervisor lifecycle passed\n";
    return 0;
  }
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world"; goal.pose.position.x = 10;
  goal.pose.position.z = 1; goal.pose.orientation.w = 1;
  goal_pub.publish(goal);
  if (!wait([&] { return latest.phase_str == "planning"; }, 2)) {
    std::cerr << "goal did not enter planning: " << latest.reason << '\n'; return 1;
  }
  std_msgs::String nav;
  if (!deadline_test) {
    nav.data = "FAILED " + std::to_string(latest.task_epoch) + " 0 IDLE";
    nav_pub.publish(nav);
    wait([] { return false; }, .12);
    if (latest.task_result_str == "failed") {
      std::cerr << "stale previous-goal failure retired the new goal\n"; return 1;
    }
  }
  nav.data = std::string(deadline_test ? "GENERATE_TRAJ " : "FAILED ") +
             std::to_string(latest.task_epoch) + " 1 " +
             (deadline_test ? "ACTIVE" : "IDLE");
  nav_pub.publish(nav);
  if (!wait([&] { return latest.task_result_str == "failed"; }, 3)) {
    std::cerr << "failure was not detected\n"; return 1;
  }
  const auto canceled_epoch = latest.task_epoch;
  // A stable vehicle alone must not unlock a still-running optimizer.
  nav.data = "FAILED " + std::to_string(canceled_epoch) + " 1 IDLE BUSY";
  nav_pub.publish(nav);
  wait([] { return false; }, 1.0);
  if (latest.ready_for_new_task) {
    std::cerr << "busy optimizer exposed readiness\n"; return 1;
  }
  nav.data = "FAILED " + std::to_string(canceled_epoch - 1) + " 1 IDLE QUIESCENT";
  nav_pub.publish(nav);
  wait([] { return false; }, .15);
  if (latest.ready_for_new_task) {
    std::cerr << "stale epoch unlocked recovery\n"; return 1;
  }
  nav.data = "FAILED " + std::to_string(canceled_epoch) + " 1 IDLE QUIESCENT";
  nav_pub.publish(nav);
  if (!wait([&] { return latest.task_epoch > canceled_epoch &&
                        latest.phase_str == "waiting_input"; }, 2)) {
    std::cerr << "recovery did not ARM a new epoch: " << latest.reason << '\n'; return 1;
  }
  if (latest.ready_for_new_task) {
    std::cerr << "ready before ARM acknowledgement\n"; return 1;
  }
  const std::string failure_reason = latest.reason;
  nav.data = "WAIT_GOAL " + std::to_string(latest.task_epoch) + " 1 IDLE QUIESCENT";
  nav_pub.publish(nav);
  if (!wait([&] { return latest.active_mode_str == "state2state" &&
                        latest.task_result_str == "failed" && latest.ready_for_new_task; }, 2)) {
    std::cerr << "failure did not recover to ready state2state: " << latest.reason << '\n'; return 1;
  }
  const std::string expected = deadline_test ? "deadline exceeded" : "retries exhausted";
  if (failure_reason.find(expected) == std::string::npos) {
    std::cerr << "wrong failure path: " << latest.reason << '\n'; return 1;
  }
  nav.data = "FAILED " + std::to_string(canceled_epoch) + " 1 IDLE QUIESCENT";
  nav_pub.publish(nav);
  wait([] { return false; }, .15);
  if (latest.task_result_str != "failed" || !latest.ready_for_new_task) {
    std::cerr << "idle status lost failure outcome\n"; return 1;
  }
  goal_pub.publish(goal);
  if (!wait([&] { return latest.phase_str == "planning" && !latest.ready_for_new_task; }, 1)) {
    std::cerr << "recovered adapter refused next goal\n"; return 1;
  }
  std::cout << "planning_failure_integration_test " << expected << " passed\n";
}
