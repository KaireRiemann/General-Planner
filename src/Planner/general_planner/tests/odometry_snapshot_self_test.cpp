#include <map_manager/map_manager.hpp>
#include <ros/callback_queue.h>

#include <atomic>
#include <cstdlib>
#include <functional>
#include <future>
#include <iostream>
#include <thread>

namespace {
class Callback : public ros::CallbackInterface {
 public:
  explicit Callback(std::function<void()> fn) : fn_(std::move(fn)) {}
  CallResult call() override { fn_(); return Success; }
 private:
  std::function<void()> fn_;
};

void expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}
}

int main() {
  // No map is attached: odometry must remain independent of fusion entirely.
  general_planner::MapManager manager;
  manager.enableIndependentOdometry();
  expect(!manager.getRobotState().rcv, "startup must wait for real odometry");

  rog_map::RobotState first{};
  first.rcv = true;
  first.rcv_time = 10.0;
  first.p.x() = 1.0;
  manager.updateOdometrySnapshot(first);
  const auto working_snapshot = manager.getRobotState();
  auto next = first;
  next.rcv_time = 10.1;
  next.p.x() = 2.0;
  manager.updateOdometrySnapshot(next);
  expect(working_snapshot.p.x() == 1.0, "in-flight plan snapshot must be immutable");
  expect(manager.getRobotState().p.x() == 2.0, "next plan must acquire latest odometry");
  for (int i = 0; i < 1000; ++i) {
    expect(manager.getRobotState().rcv_time == 10.1,
           "reads must never refresh receive timestamp after input stops");
  }
  expect(10.31 - manager.getRobotState().rcv_time > 0.2,
         "real input loss must still exceed 200 ms");

  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (int i = 1; i <= 100000; ++i) {
      rog_map::RobotState state{};
      state.rcv = true;
      state.rcv_time = i;
      state.p.setConstant(i);
      state.v.setConstant(-i);
      manager.updateOdometrySnapshot(state);
    }
    done.store(true);
  });
  while (!done.load()) {
    const auto state = manager.getRobotState();
    if (state.rcv_time == 10.1) continue;
    expect(state.p.x() == state.rcv_time && state.p.y() == state.rcv_time &&
           state.v.z() == -state.rcv_time, "concurrent readers must see a coherent snapshot");
  }
  writer.join();
  expect(manager.getRobotState().rcv_time == 100000, "latest update must be visible");

  // A blocked world callback must not prevent the dedicated odometry queue
  // from updating the snapshot. No ROS master or vehicle topics are used.
  ros::CallbackQueue world_queue, odometry_queue;
  std::promise<void> entered, release;
  auto released = release.get_future();
  world_queue.addCallback(ros::CallbackInterfacePtr(new Callback([&] {
    entered.set_value();
    released.wait();
  })));
  std::thread world([&] { world_queue.callAvailable(); });
  entered.get_future().wait();
  odometry_queue.addCallback(ros::CallbackInterfacePtr(new Callback([&] {
    manager.updateOdometrySnapshot(first);
  })));
  odometry_queue.callAvailable();
  expect(manager.getRobotState().rcv_time == first.rcv_time,
         "odometry ingress must progress while the world queue is blocked");
  release.set_value();
  world.join();
  std::cout << "odometry_snapshot_self_test passed\n";
}
