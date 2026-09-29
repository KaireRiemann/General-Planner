#!/usr/bin/env python3
"""Check source/release initialization and live frontier profile switching.

Run in the sourced ROS environment. TEST_LAUNCH_PACKAGE selects task_planner
or general_planner_release. The test owns an isolated master and publishes
stationary odometry for the normal hover gate, without a command consumer.
"""
import collections
import os
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client


with socket.socket() as reservation:
    reservation.bind(("127.0.0.1", 0))
    port = reservation.getsockname()[1]
os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
os.environ["ROS_IP"] = "127.0.0.1"
processes = []
output = collections.deque(maxlen=150)
runtime_logs = []
status = None
odom_timer = None


def start(args):
    process = subprocess.Popen(args, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True,
                               start_new_session=True)
    processes.append(process)

    def collect():
        for line in process.stdout:
            output.append(line.rstrip())

    threading.Thread(target=collect, daemon=True).start()
    return process


def wait_until(predicate, description, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if any(process.poll() is not None for process in processes):
            raise AssertionError("owned ROS process exited: " + description)
        if predicate():
            return
        time.sleep(.1)
    raise AssertionError("timeout: " + description)


try:
    start(["roscore", "-p", str(port)])
    master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])

    def master_ready():
        try:
            return master.getPid("profile_test")[0] == 1
        except OSError:
            return False

    wait_until(master_ready, "isolated master")
    import rospy
    from general_planner.msg import PlannerStatus
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Log
    from std_msgs.msg import String

    rospy.init_node("exploration_profile_test", anonymous=True, disable_signals=True)

    def on_status(message):
        global status
        status = message

    def on_log(message):
        if message.name == "/planner_runtime_node":
            runtime_logs.append(message.msg)

    rospy.Subscriber("/planner/status", PlannerStatus, on_status, queue_size=10)
    rospy.Subscriber("/rosout", Log, on_log, queue_size=200)
    request = rospy.Publisher("/planner/mode_request_text", String, queue_size=1)
    odometry = rospy.Publisher("/lidar_slam/odom", Odometry, queue_size=1)

    def publish_odom(_):
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.child_frame_id = "body"
        message.pose.pose.position.z = 1.5
        message.pose.pose.orientation.w = 1.
        odometry.publish(message)

    odom_timer = rospy.Timer(rospy.Duration(.02), publish_odom)
    package = os.environ.get("TEST_LAUNCH_PACKAGE", "task_planner")
    start(["roslaunch", package, "planner_runtime.launch",
           "initial_mode:=exploration", "exploration_mission_mode:=coverage",
           "rviz:=false", "auto_rviz_switch:=false", "traj_server:=false",
           "marsim:=false", "tracking_detector:=false", "perceptor:=false",
           "filter_mapping_cloud:=false", "odom_topic:=/lidar_slam/odom"])
    wait_until(lambda: status is not None and status.ready_for_new_task and
               status.active_mode_str == "exploration" and
               request.get_num_connections() > 0,
               "runtime heartbeat and mode receiver", timeout=60)
    prefix = "/planner_runtime_node/"
    assert rospy.get_param(prefix + "exploration/frontend") == "epicon_pointcloud"
    assert rospy.get_param(prefix + "epicon/fsm/unknown_penalty_factor") == 1.5
    assert any("[EPICON] native point-cloud frontier/topology/tour enabled" in x for x in runtime_logs)

    def runtime_pid():
        code, _, uri = master.lookupNode("profile_test", "/planner_runtime_node")
        assert code == 1
        code, _, pid = xmlrpc.client.ServerProxy(uri).getPid("profile_test")
        assert code == 1
        return pid

    pid = runtime_pid()
    wait_until(lambda: any("[frontier] granularity=indoor" in line and
                          "cluster_min_size=0.8 cluster_min_radius=1" in line
                          for line in runtime_logs), "initial indoor profile")
    for mode, profile in [("target_exploration", "default"),
                          ("exploration", "indoor")]:
        first_log = len(runtime_logs)
        request.publish(String(data=mode))
        wait_until(lambda: status.active_mode_str == mode and any(
            "[frontier] granularity=" + profile in line
            for line in runtime_logs[first_log:]), "live switch to " + mode)
        assert runtime_pid() == pid, "mode switch restarted the shared runtime"
    assert any("[frontier] granularity=indoor" in line and
               "cluster_min_size=0.8 cluster_min_radius=1" in line
               for line in runtime_logs[first_log:])
    print(package + ": heartbeat, native EPICON frontend, live mode switches: PASS")
except Exception:
    print("\n".join(output))
    print("\n".join(runtime_logs[-30:]))
    raise
finally:
    if odom_timer is not None:
        odom_timer.shutdown()
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
