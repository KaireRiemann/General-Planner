#include <general_core/planner_runtime/planner_supervisor.hpp>
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv) {
  // This fixture publishes synthetic vehicle data. Never allow the normal
  // vehicle master: run only with the explicitly isolated test master.
  const char *master = std::getenv("ROS_MASTER_URI");
  if (!master || std::string(master) != "http://127.0.0.1:11381") {
    std::cerr << "requires isolated ROS_MASTER_URI=http://127.0.0.1:11381\n";
    return 2;
  }
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
  auto nav_pub = nh.advertise<std_msgs::String>("/planning/navigation/status", 1);
  auto wait = [&](const std::function<bool()> &predicate, double seconds) {
    const auto start = ros::WallTime::now();
    while (ros::ok() && (ros::WallTime::now() - start).toSec() < seconds) {
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now(); odom.header.frame_id = "world";
      odom.pose.pose.orientation.w = 1; odom.pose.pose.position.z = 1;
      odom_pub.publish(odom); ros::spinOnce();
      if (predicate()) return true;
      ros::WallDuration(.01).sleep();
    }
    return false;
  };
  if (!wait([&] { return latest.active_mode_str == "state2state" &&
                        latest.ready_for_new_task && goal_pub.getNumSubscribers(); }, 5)) {
    std::cerr << "boot failed: " << latest.reason << '\n'; return 1;
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
