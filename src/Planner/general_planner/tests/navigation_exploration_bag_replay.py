#!/usr/bin/env python3
"""Replay navigation admission and coverage handoff with measured Unity feedback.

Use an isolated master and rebuild the map from recorded geometry. Navigation
is prearmed: recorded feedback cannot respond to the new hover gate during the
historical exploration-to-navigation handover. That gate is tested separately
by navigation_goal_admission_integration_test. No report files are created.
"""
import argparse
import collections
import json
import math
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
    parser.add_argument("--port", type=int, default=11349)
    args = parser.parse_args()
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:" + str(args.port)
    import rosbag
    import rosgraph
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from quadrotor_msgs.msg import PositionCommand
    from general_planner.msg import NavigationGoalAck, PlannerPositionCommand, PlannerStatus

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
        rospy.init_node("navigation_exploration_bag_replay", disable_signals=True)
        statuses, acks, nav_commands, exploration_commands, feedback = [], [], [], [], []
        rospy.Subscriber("/planner/status", PlannerStatus,
                         lambda m: statuses.append((rospy.Time.now().to_sec(), m)), queue_size=1000)
        rospy.Subscriber("/planning/navigation/goal_ack", NavigationGoalAck, acks.append, queue_size=100)
        rospy.Subscriber("/planning/navigation/pos_cmd", PositionCommand, nav_commands.append, queue_size=2000)
        rospy.Subscriber("/planning/exploration/command_bound", PlannerPositionCommand,
                         exploration_commands.append, queue_size=2000)
        rospy.Subscriber("/lidar_slam/odom", Odometry, feedback.append, queue_size=1000)
        clock = rospy.Publisher("/clock", Clock, queue_size=1, latch=True)
        publishers = {
            "/unity_odom_walltime": rospy.Publisher("/bag_replay/unity_odom", Odometry, queue_size=10),
            "/drone_0_pcl_render_node/cloud_walltime": rospy.Publisher("/cloud_registered", PointCloud2, queue_size=10),
            "/planner/mode_request_text": rospy.Publisher("/planner/mode_request_text", String, queue_size=10),
            "/goal_3d": rospy.Publisher("/goal_3d", PoseStamped, queue_size=10),
            "/move_base_simple/goal": rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10),
        }
        repo = Path(__file__).resolve().parents[4]
        bridge = repo / "general_planner_release/src/unity_planner_bridge/scripts/unity_cmd_odom_bridge.py"
        spawn(["python3", str(bridge), "__name:=bag_replay_feedback_bridge",
               "_unity_feedback_odom_topic:=/bag_replay/unity_odom", "_estimate_feedback_twist:=true",
               "_unity_odom_topics:=/bag_replay/command_odom",
               "_unity_cloud_topics:=/bag_replay/unused_cloud"])
        config = Path(__file__).resolve().parents[1] / "config"
        runtime = spawn(["roslaunch", "general_planner_release" if args.release else "task_planner",
                         "planner_runtime.launch", "initial_mode:=state2state",
                         "exploration_mission_mode:=coverage", "tracking_detector:=false",
                         "perceptor:=false", "rviz:=false", "auto_rviz_switch:=false",
                         "cloud_odom_mode:=latest_odom", "source_startup_grace_duration:=2.0",
                         "exploration_config:=" + str(config / "exploration_house.yaml"),
                         "exploration_overlay_config:=" + str(config / "exploration_unity_overlay.yaml"),
                         "tracking_rog_map_config:=" + str(config / "tracking_unity_rog_map.yaml"),
                         "rog_map_config:=" + str(config / "exploration_unity_neighborhood_rog_map.yaml")])
        for _ in range(400):
            if all(publishers[t].get_num_connections() for t in publishers):
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
                # The initial latched RViz goal predates this recording by ten
                # minutes. Replaying it as a new click invents an extra task.
                if topic == "/move_base_simple/goal" and stamp.to_sec() - start < 1:
                    continue
                publishers[topic].publish(message)
                count[topic] += 1
        time.sleep(.3)
        completed = [s for t, s in statuses if 18 < t-start < 24.14 and
                     s.active_mode_str == "state2state" and s.task_result_str == "succeeded"]
        rejected = [m for m in acks if m.result == m.REJECTED and m.reason == "too_close"]
        timeouts = [s.reason for _, s in statuses if "command source timeout" in s.reason or
                    "first-command deadline exceeded" in s.reason or "admission timeout" in s.reason]
        nonzero_feedback = sum(math.sqrt(m.twist.twist.linear.x**2 + m.twist.twist.linear.y**2 +
                                        m.twist.twist.linear.z**2) > .1 for m in feedback)
        print(json.dumps({"release": args.release, "feedback": "recorded Unity pose through real velocity bridge",
                          "map": "rebuilt from recorded wall-time clouds", "navigation_prearmed": True, "input_counts": count,
                          "navigation_commands": len(nav_commands), "too_close_acknowledgements": len(rejected),
                          "goal_acknowledgements": [{"request": a.request_id, "result": a.result, "sequence": a.goal_sequence, "reason": a.reason} for a in acks],
                          "accepted_goal_completed_after_rejection": bool(completed),
                          "task_bound_exploration_commands": len(exploration_commands),
                          "measured_moving_feedback_samples": nonzero_feedback,
                          "source_or_admission_timeouts": len(timeouts)}, indent=2), flush=True)
        if not rejected or not completed or not exploration_commands or not nonzero_feedback or timeouts:
            raise RuntimeError("navigation/exploration lifecycle replay failed")
    except Exception:
        for log in logs:
            log.seek(0)
            lines = log.read().splitlines()
            interesting = [line for line in lines if any(word in line for word in
                           ("ERROR", "Traceback", "Exception", "discard", "rejected", "authorize", "deadline", "drop navigation", "armed"))]
            print("\n".join((interesting or lines)[-25:]), flush=True)
        raise
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
