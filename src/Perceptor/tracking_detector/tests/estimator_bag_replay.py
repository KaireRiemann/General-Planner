#!/usr/bin/env python3
"""Deterministic, offline estimator regression using recorded input arrival times.

Ground truth is read only for scoring. No ROS master, publishers, or output files.
"""
import argparse
import importlib.util
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np
import rosbag
import rospy
import yaml


def load(path):
    spec = importlib.util.spec_from_file_location("replay_estimator", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def replay(path, events, end, epoch, parameters, truth, windows=()):
    module = load(path)
    clock = [epoch]
    published = {}

    class Publisher:
        def __init__(self, topic, *args, **kwargs):
            self.topic = topic
            published[topic] = []

        def publish(self, message):
            published[self.topic].append((clock[0]-epoch, message))

    with patch.object(module.rospy, "get_param", side_effect=lambda key, default=None: parameters.get(key.lstrip("~"), default)), \
         patch.object(module.rospy, "Publisher", Publisher), \
         patch.object(module.rospy, "Subscriber"), patch.object(module.rospy, "Timer"), \
         patch.object(module.rospy, "on_shutdown"), \
         patch.object(module.rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(clock[0])), \
         patch.object(module.time, "monotonic", side_effect=lambda: clock[0]):
        node = module.TargetEstimator()
        callbacks = {"/unity_odom": node.on_odom, "/tracking/bboxes": node.on_bbox,
                     "/camera0/color/info": node.on_info}
        if hasattr(node, "on_capture_pose"):
            callbacks["/camera0/capture_pose"] = node.on_capture_pose
        next_tick = 0.
        for t, topic, message in events:
            while next_tick < t:
                clock[0] = epoch+next_tick
                node.tick(None)
                next_tick += .05
            clock[0] = epoch+t
            if topic in callbacks:
                callbacks[topic](message)
        while next_tick <= end:
            clock[0] = epoch+next_tick
            node.tick(None)
            next_tick += .05

    states = [(t,json.loads(m)) for t,m in published["~status"]]
    states_by_time = {round(t,4): s for t,s in states}
    errors, velocity_errors, scored_times = [], [], []
    for t,m in published["~target_odom"]:
        s = states_by_time[round(t,4)]
        sensor = s["observation_stamp"]+s["observation_age"]
        if sensor < truth[0,0] or sensor > truth[-1,0]:
            continue
        gt = np.array([np.interp(sensor,truth[:,0],truth[:,j]) for j in (1,2)])
        position = np.array([m.pose.pose.position.x,m.pose.pose.position.y])
        errors.append(np.linalg.norm(position-gt))
        scored_times.append(t)
        i = min(len(truth)-1,max(1,np.searchsorted(truth[:,0],sensor)))
        velocity = (truth[i,1:3]-truth[i-1,1:3])/(truth[i,0]-truth[i-1,0])
        estimated = np.array([m.twist.twist.linear.x,m.twist.twist.linear.y])
        velocity_errors.append(np.linalg.norm(estimated-velocity))
    valid_seconds = sum(states[i+1][0]-t for i,(t,s) in enumerate(states[:-1]) if s["valid"])
    losses = sum(a[1]["valid"] and not b[1]["valid"] for a,b in zip(states,states[1:]))
    result = dict(valid_seconds=valid_seconds, losses=losses, outputs=len(errors),
                loss_events=[(round(b[0],3),b[1]["validity_reason"]) for a,b in zip(states,states[1:])
                             if a[1]["valid"] and not b[1]["valid"]],
                position_error_p50=float(np.median(errors)) if errors else None,
                position_error_p90=float(np.percentile(errors,90)) if errors else None,
                velocity_rmse=float(np.sqrt(np.mean(np.square(velocity_errors)))) if errors else None)
    for begin,finish in windows:
        indices=[i for i,t in enumerate(scored_times) if begin<=t<finish]
        result["window_%g_%g"%(begin,finish)]=dict(outputs=len(indices),
            position_error_p50=float(np.median([errors[i] for i in indices])) if indices else None,
            velocity_rmse=float(np.sqrt(np.mean([velocity_errors[i]**2 for i in indices]))) if indices else None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--same-parameters", action="store_true")
    parser.add_argument("--window", action="append", default=[], help="score interval in bag seconds, start:end")
    args = parser.parse_args()
    windows=[tuple(map(float,value.split(":"))) for value in args.window]
    package = Path(__file__).resolve().parents[1]
    parameters = yaml.safe_load((package/"config/estimator.yaml").read_text())
    parameters.update(cam2body_p=[0,0,0],cam2body_R=[0,0,1,-1,0,0,0,-1,0],
                      ground_z=0.,target_center_height=.7,range_method="auto")
    input_topics = ["/unity_odom","/tracking/bboxes","/camera0/color/info","/camera0/capture_pose"]
    events, truth = [], []
    with rosbag.Bag(str(args.bag)) as bag:
        epoch, end = bag.get_start_time(), bag.get_end_time()-bag.get_start_time()
        for topic,m,t in bag.read_messages(topics=input_topics+["/unity/car_ground_truth/odom"]):
            if topic == "/unity/car_ground_truth/odom":
                truth.append([m.header.stamp.to_sec(),m.pose.pose.position.x,m.pose.pose.position.y])
            else:
                events.append((t.to_sec()-epoch,topic,m))
    truth = np.asarray(truth)
    if len(truth) < 2:
        raise RuntimeError("bag needs native-clock car ground truth for scoring")
    result = replay(package/"scripts/target_state_estimator.py",events,end,epoch,parameters,truth,windows)
    print(json.dumps({"current":result},indent=2))
    if args.baseline:
        old_parameters = parameters if args.same_parameters else dict(parameters,range_method="ground_plane",max_detection_gap=.45)
        baseline = replay(args.baseline,events,end,epoch,old_parameters,truth,windows)
        print(json.dumps({"baseline":baseline},indent=2))
        if args.same_parameters:
            return
        assert result["valid_seconds"] > baseline["valid_seconds"]
        assert result["position_error_p90"] < baseline["position_error_p90"]
        assert result["velocity_rmse"] < baseline["velocity_rmse"]


if __name__ == "__main__":
    main()
