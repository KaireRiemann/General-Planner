#!/usr/bin/env python3
"""Validate tracking planning against recorded geometry on an isolated master.

Publishes the bag's wall-time cloud/vehicle pose and recorded target odometry.
The tracking map is rebuilt, and vehicle feedback is open-loop. No analysis
files are created and no commands are sent to an existing ROS master.
"""
import argparse
import collections
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--release", action="store_true")
    parser.add_argument("--port", type=int, default=11339)
    args = parser.parse_args()
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:" + str(args.port)
    import rosbag
    import rosgraph
    import rospy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from quadrotor_msgs.msg import PositionCommand
    from general_planner.msg import PlannerStatus

    try:
        rosgraph.Master("/probe").getPid()
    except Exception:
        pass
    else:
        raise RuntimeError("isolated master port already in use")
    processes, logs = [], []
    def spawn(command):
        log = tempfile.TemporaryFile(mode="w+")
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    try:
        spawn(["roscore", "-p", str(args.port)])
        for _ in range(100):
            try:
                rosgraph.Master("/probe").getPid()
                break
            except Exception:
                time.sleep(.05)
        rospy.set_param("/use_sim_time", True)
        rospy.init_node("tracking_corridor_bag_replay", disable_signals=True)
        events, statuses, source_commands = [], [], []
        rospy.Subscriber("/planning/diagnostics/events", String,
                         lambda m: events.append(m.data), queue_size=10000)
        rospy.Subscriber("/planner/status", PlannerStatus, statuses.append, queue_size=1000)
        rospy.Subscriber("/planning/navigation/pos_cmd", PositionCommand,
                         source_commands.append, queue_size=2000)
        clock = rospy.Publisher("/clock", Clock, queue_size=10)
        publishers = {
            "/unity_odom_walltime": rospy.Publisher("/lidar_slam/odom", Odometry, queue_size=10),
            "/drone_0_pcl_render_node/cloud_walltime": rospy.Publisher("/cloud_registered", PointCloud2, queue_size=10),
            "/tracking/target_odom": rospy.Publisher("/tracking/target_odom", Odometry, queue_size=10),
        }
        config = Path(__file__).resolve().parents[1] / "config"
        runtime = spawn(["roslaunch", "general_planner_release" if args.release else "task_planner",
                         "planner_runtime.launch", "initial_mode:=tracking", "tracking_detector:=false",
                         "perceptor:=false", "rviz:=false", "auto_rviz_switch:=false",
                         "enable_exploration:=false", "source_startup_grace_duration:=2.0",
                         "cloud_odom_mode:=latest_odom", "tracking_target_odom_topic:=/tracking/target_odom",
                         "tracking_rog_map_config:=" + str(config / "tracking_unity_rog_map.yaml"),
                         "rog_map_config:=" + str(config / "exploration_unity_neighborhood_rog_map.yaml")])
        for _ in range(300):
            subscribers = dict(rospy.get_master().getSystemState()[2][1])
            if "/tracking/target_odom" in subscribers and "/cloud_registered" in subscribers:
                break
            if runtime.poll() is not None:
                raise RuntimeError("runtime exited during startup")
            time.sleep(.05)
        else:
            raise RuntimeError("runtime input subscribers did not start")
        count = collections.Counter()
        with rosbag.Bag(str(args.bag)) as bag:
            start = bag.get_start_time()
            wall_start = time.monotonic()
            for topic, message, stamp in bag.read_messages(topics=list(publishers)):
                if runtime.poll() is not None:
                    raise RuntimeError("runtime exited during replay")
                remaining = wall_start + stamp.to_sec() - start - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                clock.publish(Clock(clock=stamp))
                publishers[topic].publish(message)
                count[topic] += 1
        time.sleep(.3)
        parsed = []
        for event in events:
            fields = dict(part.split("=", 1) for part in event.split(";") if "=" in part)
            if "detail" in fields and "=" in fields["detail"]:
                key, value = fields["detail"].split("=", 1)
                fields[key] = value
            parsed.append(fields)
        contexts = [e for e in parsed if e.get("event") in
                    ("tracking_plan_from_rest_context", "tracking_replan_context")]
        failures = [s.reason for s in statuses if "command source timeout" in s.reason]
        successes = sum(e.get("phase") == "success" for e in contexts)
        corridor_failures = sum(e.get("phase") == "corridor" for e in contexts)
        print(json.dumps({"release": args.release, "feedback": "recorded open-loop pose",
                          "map": "rebuilt from recorded wall-time clouds",
                          "input_counts": count, "navigation_source_commands": len(source_commands),
                          "successful_plans": successes,
                          "corridor_failures": corridor_failures,
                          "planning_phases": collections.Counter(e.get("phase") for e in contexts),
                          "source_timeouts": len(failures)}, indent=2), flush=True)
        if not source_commands or not successes or failures or corridor_failures:
            failed = [e for e in contexts if e.get("phase") != "success"]
            for e in (failed or contexts)[:3]:
                print("first_context:", e.get("phase"), e.get("reason"), flush=True)
            raise RuntimeError("tracking replay failed corridor/command regression checks")
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
