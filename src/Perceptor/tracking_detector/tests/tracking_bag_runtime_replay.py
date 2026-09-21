#!/usr/bin/env python3
"""Replay recorded sensors through estimator/predictor/planner on an owned master.

Feedback is recorded, not a new closed-loop simulation. Check execution lifecycle
and node stability here; use tracking_runtime_command_smoke.py for closed-loop checks.
"""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:11329"
import rosgraph
import rosnode
import rospy
from general_planner.msg import PlannerStatus
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag",type=Path)
    parser.add_argument("--release",action="store_true")
    args = parser.parse_args()
    try:
        rosgraph.Master("/probe").getPid()
    except Exception:
        pass
    else:
        raise RuntimeError("isolated master 11329 is already in use")
    processes, logs = [], []

    def spawn(command,name):
        log = open("/tmp/tracking_bag_regression_"+name+".log","w")
        logs.append(log)
        process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
        return process

    try:
        spawn(["roscore","-p","11329"],"master")
        for _ in range(100):
            try:
                rosgraph.Master("/probe").getPid()
                break
            except Exception:
                time.sleep(.05)
        rospy.set_param("/use_sim_time",True)
        rospy.init_node("tracking_bag_regression")
        commands, events, statuses = [], [], []
        rospy.Subscriber("/planning/pos_cmd",PositionCommand,commands.append,queue_size=2000)
        rospy.Subscriber("/planning/diagnostics/events",String,lambda m:events.append(m.data),queue_size=1000)
        rospy.Subscriber("/planner/status",PlannerStatus,statuses.append,queue_size=500)
        package = "general_planner_release" if args.release else "task_planner"
        runtime = spawn(["roslaunch",package,"planner_runtime.launch","initial_mode:=tracking",
                         "tracking_detector:=false","perceptor:=false","rviz:=false",
                         "auto_rviz_switch:=false","enable_exploration:=false"],"runtime")
        detector = spawn(["roslaunch","tracking_detector","tracking_detector.launch",
                          "start_detector:=false","range_method:=auto",
                          "camera_config:="+str(Path(__file__).resolve().parents[1]/"config/camera_unity_main_scene.yaml")],"estimator")
        for _ in range(200):
            subscriptions = dict(rospy.get_master().getSystemState()[2][1])
            if "/tracking/target_prediction" in subscriptions and "/tracking/bboxes" in subscriptions:
                break
            assert runtime.poll() is None and detector.poll() is None,"startup failed"
            time.sleep(.05)
        else:
            raise RuntimeError("input subscribers did not start")
        topics = ["/unity_odom","/lidar_slam/odom","/cloud_registered","/tracking/bboxes",
                  "/camera0/color/info","/camera0/capture_pose"]
        player = spawn(["rosbag","play","--clock","--delay=1",str(args.bag),"--topics"]+topics,"player")
        deadline = time.monotonic()+180
        while player.poll() is None and time.monotonic()<deadline:
            assert runtime.poll() is None and detector.poll() is None,"node exited during replay"
            time.sleep(.1)
        assert player.poll()==0,"bag player failed or timed out"
        time.sleep(.3)
        assert rosnode.rosnode_ping("/planner_runtime_node",max_count=1,verbose=False),"planner runtime died"
        assert rosnode.rosnode_ping("/tracking_detector/target_ekf",max_count=1,verbose=False),"estimator died"
        parsed = [dict(f.split("=",1) for f in message.split(";") if "=" in f) for message in events]
        contexts = [e for e in parsed if e.get("event") in ("tracking_plan_from_rest_context","tracking_replan_context")]
        keep_counts = [int(e.get("keep_old_count",0)) for e in contexts]
        assert all(b <= a+1 for a,b in zip(keep_counts,keep_counts[1:])),"keep-old was counted more than once per attempt"
        assert len(statuses)>20 and len(commands)>100,"runtime did not execute"
        assert any(e.get("phase")=="success" for e in contexts),"no tracking trajectory committed"
        assert not any(e.get("event")=="tracking_plan_from_rest_context" and e.get("phase")=="keep_old"
                       for e in contexts),"interrupted trajectory reused on reacquisition"
        assert all(math.isfinite(value) for c in commands for value in
                   (c.position.x,c.position.y,c.position.z,c.velocity.x,c.velocity.y,c.velocity.z,c.yaw,c.yaw_dot))
        assert all(e.get("ret_code_name") in ("NO_NEED","FAILED","OPT_FAILED") for e in contexts
                   if e.get("phase") in ("keep_old","recovery_hold","recovery_continue")),"misleading success diagnostic"
        print(json.dumps(dict(result="PASS",release=args.release,commands=len(commands),
                              planning_phases=collections.Counter(e.get("phase") for e in contexts),
                              maximum_keep_old_count=max(keep_counts,default=0),
                              final_phase=statuses[-1].phase_str,
                              final_reason=statuses[-1].reason),indent=2),flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
