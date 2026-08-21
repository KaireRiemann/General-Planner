#include <general_core/planner_runtime/planner_command_gateway.hpp>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using general_planner::planner_runtime::CommandOwner;
using general_planner::planner_runtime::PlannerCommandGateway;
using quadrotor_msgs::PositionCommand;

class OutputCapture {
 public:
  void callback(const PositionCommand::ConstPtr &message) {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    messages_.push_back(*message);
    condition_.notify_all();
  }

  template <typename Predicate>
  bool waitFor(const Predicate &predicate, const double timeout_seconds) {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
        lock, std::chrono::duration<double>(timeout_seconds), [&] {
          return std::any_of(messages_.begin(), messages_.end(), predicate);
        });
  }

  std::size_t messageCount() {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_.size();
  }

  bool waitForNoNewMessages(const std::size_t initial_count,
                            const double timeout_seconds) {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool received_new_message = condition_.wait_for(
        lock, std::chrono::duration<double>(timeout_seconds), [&] {
          return messages_.size() > initial_count;
        });
    return !received_new_message;
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<PositionCommand> messages_;
};

void expect(const bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool positionNear(const geometry_msgs::Point &position, const double x,
                  const double y, const double z) {
  constexpr double kTolerance = 1.0e-3;
  return std::fabs(position.x - x) <= kTolerance &&
         std::fabs(position.y - y) <= kTolerance &&
         std::fabs(position.z - z) <= kTolerance;
}

template <typename Publisher>
bool waitForSubscriber(const Publisher &publisher,
                       const double timeout_seconds) {
  const ros::WallTime deadline = ros::WallTime::now() +
                                 ros::WallDuration(timeout_seconds);
  while (ros::ok() && ros::WallTime::now() < deadline) {
    if (publisher.getNumSubscribers() > 0) {
      return true;
    }
    ros::WallDuration(0.01).sleep();
  }
  return false;
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_command_gateway_behavior_self_test");
  int result = 0;
  try {
    ros::NodeHandle params("~");
    const std::string navigation_topic =
        "/planner_command_gateway_behavior_self_test/navigation";
    const std::string output_topic =
        "/planner_command_gateway_behavior_self_test/output";
    const std::string odom_topic =
        "/planner_command_gateway_behavior_self_test/odom";
    params.setParam("navigation_cmd_topic", navigation_topic);
    params.setParam("exploration_cmd_topic",
                    "/planner_command_gateway_behavior_self_test/exploration");
    params.setParam("output_cmd_topic", output_topic);
    params.setParam("odometry_topic", odom_topic);
    params.setParam("command_timeout", 0.05);
    params.setParam("publish_rate", 100.0);

    ros::NodeHandle bus;
    OutputCapture output_capture;
    const ros::Subscriber output_subscriber = bus.subscribe<PositionCommand>(
        output_topic, 50, &OutputCapture::callback, &output_capture);
    const ros::Publisher navigation_publisher =
        bus.advertise<PositionCommand>(navigation_topic, 1);
    const ros::Publisher odom_publisher = bus.advertise<nav_msgs::Odometry>(
        odom_topic, 1);

    PlannerCommandGateway gateway(params);
    // Reproduce the old failure setup: a valid explicit hold anchor belongs
    // to a previous task, then state2state becomes the authorized owner.
    gateway.lockHoldAnchor(-64.15, 8.94, 1.70, 0.0);
    gateway.setAuthorizedOwner(CommandOwner::STATE2STATE, 1);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    expect(waitForSubscriber(navigation_publisher, 2.0),
           "gateway did not subscribe to navigation input");
    expect(waitForSubscriber(odom_publisher, 2.0),
           "gateway did not subscribe to odometry input");

    nav_msgs::Odometry odom;
    odom.header.frame_id = "world";
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.pose.position.x = 10.0;
    odom.pose.pose.position.y = 2.0;
    odom.pose.pose.position.z = 3.0;
    odom_publisher.publish(odom);

    expect(output_capture.waitFor(
               [](const PositionCommand &command) {
                 return positionNear(command.position, 10.0, 2.0, 3.0);
               },
               2.0),
           "stale state2state source did not hold current odometry");
    expect(!output_capture.waitFor(
               [](const PositionCommand &command) {
                 return positionNear(command.position, -64.15, 8.94, 1.70);
               },
               0.10),
           "stale state2state source replayed the old task hold anchor");

    PositionCommand navigation;
    navigation.header.frame_id = "world";
    navigation.trajectory_flag = PositionCommand::TRAJECTORY_STATUS_READY;
    navigation.trajectory_id = 1001;
    navigation.position.x = 10.5;
    navigation.position.y = 2.0;
    navigation.position.z = 3.0;
    navigation_publisher.publish(navigation);
    expect(output_capture.waitFor(
               [](const PositionCommand &command) {
                 return command.trajectory_id == 1001 &&
                        positionNear(command.position, 10.5, 2.0, 3.0);
               },
               2.0),
           "fresh state2state command did not resume after timeout hold");

    // Gate is an external-control handover, not an explicit HOLD. Even if
    // publishing_enabled were accidentally left true, the gateway policy must
    // emit no PositionCommand while CommandOwner::GATE is authorized.
    const std::size_t output_count_before_gate = output_capture.messageCount();
    gateway.setAuthorizedOwner(CommandOwner::GATE, 2);
    expect(output_capture.waitForNoNewMessages(output_count_before_gate, 0.15),
           "gate owner unexpectedly published a position command");

    spinner.stop();
    std::cout << "planner_command_gateway_behavior_self_test passed\n";
  } catch (const std::exception &error) {
    result = 1;
    std::cerr << "planner_command_gateway_behavior_self_test failed: "
              << error.what() << '\n';
  }
  ros::shutdown();
  return result;
}
