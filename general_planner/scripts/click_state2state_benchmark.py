#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from threading import Lock

import rosgraph
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PolynomialTrajectory
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


DEMO_LAUNCHES = {
    "click": "click_demo.launch",
    "smooth": "click_demo.launch",
    "esdf": "click_esdf_demo.launch",
    "plain": "click_plain_demo.launch",
}

CSV_FIELDS = [
    "demo",
    "trial",
    "success",
    "failure_reason",
    "goal_x",
    "goal_y",
    "goal_z",
    "start_x",
    "start_y",
    "start_z",
    "goal_distance_m",
    "sampling_phase",
    "required_goal_distance_m",
    "nearest_recent_goal_distance_m",
    "sample_attempts",
    "goal_clearance_m",
    "first_plan_latency_ms",
    "elapsed_s",
    "trajectory_count",
    "replan_count",
    "first_traj_id",
    "last_traj_id",
    "first_traj_duration_s",
    "first_traj_length_m",
    "first_traj_avg_speed_mps",
    "first_traj_max_speed_mps",
    "last_traj_duration_s",
    "last_traj_length_m",
    "last_traj_avg_speed_mps",
    "last_traj_max_speed_mps",
    "executed_length_m",
    "executed_avg_speed_mps",
    "planned_collision_traj_count",
    "planned_collision_sample_count",
    "executed_collision_sample_count",
    "collision",
]


@dataclass
class TrajectoryEvent:
    msg: PolynomialTrajectory
    wall_time: float
    ros_time: float


def parse_args():
    parser = argparse.ArgumentParser(
        description="Autonomous benchmark for state-to-state click demos."
    )
    parser.add_argument(
        "--demo",
        default="all",
        help="click/smooth, esdf, plain, all, or a comma-separated list.",
    )
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--x-min", type=float, default=-20.0)
    parser.add_argument("--x-max", type=float, default=20.0)
    parser.add_argument("--y-min", type=float, default=-20.0)
    parser.add_argument("--y-max", type=float, default=20.0)
    parser.add_argument("--goal-z", type=float, default=1.5)
    parser.add_argument("--min-goal-distance", type=float, default=2.0)
    parser.add_argument("--warmup-trials", type=int, default=5)
    parser.add_argument("--warmup-radius", type=float, default=4.0)
    parser.add_argument("--wide-min-goal-distance", type=float, default=6.0)
    parser.add_argument("--min-recent-goal-distance", type=float, default=6.0)
    parser.add_argument("--recent-goal-window", type=int, default=12)
    parser.add_argument("--candidate-batch", type=int, default=32)
    parser.add_argument("--goal-clearance", type=float, default=0.45)
    parser.add_argument("--collision-clearance", type=float, default=0.25)
    parser.add_argument("--map-voxel", type=float, default=0.10)
    parser.add_argument("--trajectory-sample-dt", type=float, default=0.05)
    parser.add_argument("--odom-sample-dt", type=float, default=0.05)
    parser.add_argument("--goal-tolerance", type=float, default=0.35)
    parser.add_argument("--speed-tolerance", type=float, default=0.35)
    parser.add_argument("--plan-timeout", type=float, default=8.0)
    parser.add_argument("--trial-timeout", type=float, default=60.0)
    parser.add_argument("--startup-wait", type=float, default=2.0)
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--goal-publish-time", type=float, default=1.0)
    parser.add_argument("--goal-publish-rate", type=float, default=20.0)
    parser.add_argument("--sample-max-attempts", type=int, default=5000)
    parser.add_argument("--rviz", type=str_to_bool, default=False)
    parser.add_argument("--fpv-rviz", type=str_to_bool, default=False)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def str_to_bool(value):
    if isinstance(value, bool):
        return value
    value = value.lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Expected a boolean value, got '{value}'.")


def expand_demos(demo_arg):
    if demo_arg == "all":
        return ["click", "esdf", "plain"]
    demos = []
    for item in demo_arg.split(","):
        key = item.strip().lower()
        if not key:
            continue
        if key not in DEMO_LAUNCHES:
            raise ValueError(f"Unsupported demo '{key}'. Use click, esdf, plain, or all.")
        key = "click" if key == "smooth" else key
        if key not in demos:
            demos.append(key)
    return demos


def ensure_ros_master():
    master = rosgraph.Master("/click_state2state_benchmark")
    try:
        master.getPid()
        return None
    except Exception:
        proc = subprocess.Popen(
            ["roscore"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        deadline = time.time() + 15.0
        while time.time() < deadline:
            try:
                master.getPid()
                return proc
            except Exception:
                time.sleep(0.2)
        stop_process_group(proc, "roscore")
        raise RuntimeError("Timed out while starting roscore.")


def stop_process_group(proc, name):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=10.0)
        return
    except Exception:
        pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5.0)
        return
    except Exception:
        pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception as exc:
        rospy.logwarn("Failed to stop %s: %s", name, exc)


def point_distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) * (a[0] - b[0])
        + (a[1] - b[1]) * (a[1] - b[1])
        + (a[2] - b[2]) * (a[2] - b[2])
    )


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def safe_float(value):
    if value is None:
        return ""
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return ""
        return f"{value:.6f}"
    return value


class OccupancyIndex:
    def __init__(self, cloud_msg, voxel_size):
        self.voxel_size = max(0.03, voxel_size)
        self.occupied = set()
        self.bounds = [float("inf"), float("inf"), float("inf"),
                       -float("inf"), -float("inf"), -float("inf")]
        self.point_count = 0
        for x, y, z in point_cloud2.read_points(
            cloud_msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y, z = float(x), float(y), float(z)
            self.point_count += 1
            self.bounds[0] = min(self.bounds[0], x)
            self.bounds[1] = min(self.bounds[1], y)
            self.bounds[2] = min(self.bounds[2], z)
            self.bounds[3] = max(self.bounds[3], x)
            self.bounds[4] = max(self.bounds[4], y)
            self.bounds[5] = max(self.bounds[5], z)
            self.occupied.add(self.key((x, y, z)))
        if not self.occupied:
            raise RuntimeError("The global point cloud is empty; cannot sample safe goals.")
        self._offset_cache = {}

    def key(self, p):
        inv = 1.0 / self.voxel_size
        return (
            int(math.floor(p[0] * inv)),
            int(math.floor(p[1] * inv)),
            int(math.floor(p[2] * inv)),
        )

    def offsets(self, clearance):
        radius = int(math.ceil((clearance + math.sqrt(3.0) * self.voxel_size) / self.voxel_size))
        if radius in self._offset_cache:
            return self._offset_cache[radius]
        offsets = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
                    offsets.append((dx, dy, dz))
        self._offset_cache[radius] = offsets
        return offsets

    def clearance(self, p, max_distance):
        key = self.key(p)
        max_sq = max_distance * max_distance
        best_sq = max_sq
        found = False
        for dx, dy, dz in self.offsets(max_distance):
            nk = (key[0] + dx, key[1] + dy, key[2] + dz)
            if nk not in self.occupied:
                continue
            cx = (nk[0] + 0.5) * self.voxel_size
            cy = (nk[1] + 0.5) * self.voxel_size
            cz = (nk[2] + 0.5) * self.voxel_size
            dsq = (p[0] - cx) ** 2 + (p[1] - cy) ** 2 + (p[2] - cz) ** 2
            if dsq < best_sq:
                best_sq = dsq
                found = True
        return math.sqrt(best_sq) if found else max_distance

    def is_free(self, p, clearance):
        return self.clearance(p, clearance) >= clearance


class BenchmarkNode:
    def __init__(self, args):
        self.args = args
        self.lock = Lock()
        self.latest_odom = None
        self.latest_cloud = None
        self.odom_seq = 0
        self.cloud_seq = 0
        self.traj_events = []
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber("/lidar_slam/odom", Odometry, self.odom_cb, queue_size=50)
        self.cloud_sub = rospy.Subscriber("/global_pc", PointCloud2, self.cloud_cb, queue_size=1)
        self.traj_sub = rospy.Subscriber(
            "/planning_cmd/poly_traj", PolynomialTrajectory, self.traj_cb, queue_size=100
        )

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg
            self.odom_seq += 1

    def cloud_cb(self, msg):
        with self.lock:
            self.latest_cloud = msg
            self.cloud_seq += 1

    def traj_cb(self, msg):
        if (msg.type & PolynomialTrajectory.POSITION_TRAJ) == 0:
            return
        if msg.piece_num_pos <= 0 or len(msg.time_pos) == 0:
            return
        event = TrajectoryEvent(msg=msg, wall_time=time.monotonic(), ros_time=rospy.Time.now().to_sec())
        with self.lock:
            self.traj_events.append(event)

    def reset_trial_events(self):
        with self.lock:
            self.traj_events = []

    def get_odom_state(self):
        with self.lock:
            msg = self.latest_odom
        if msg is None:
            return None
        p = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        v = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        return p, math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    def copy_traj_events(self):
        with self.lock:
            return list(self.traj_events)

    def wait_for_ready(self, previous_cloud_seq, previous_odom_seq):
        deadline = time.monotonic() + 30.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                odom_ready = self.latest_odom is not None and self.odom_seq > previous_odom_seq
                cloud_ready = self.latest_cloud is not None and self.cloud_seq > previous_cloud_seq
            if odom_ready and cloud_ready and self.goal_pub.get_num_connections() > 0:
                return True
            time.sleep(0.1)
        return False

    def current_cloud_seq(self):
        with self.lock:
            return self.cloud_seq

    def current_odom_seq(self):
        with self.lock:
            return self.odom_seq

    def latest_cloud_msg(self):
        with self.lock:
            return self.latest_cloud

    def publish_goal(self, goal, start):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        count = max(1, int(self.args.goal_publish_time * self.args.goal_publish_rate))
        rate = rospy.Rate(self.args.goal_publish_rate)
        for _ in range(count):
            if rospy.is_shutdown():
                return
            msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(msg)
            rate.sleep()


def eval_poly(coeffs, t):
    value = 0.0
    for c in coeffs:
        value = value * t + c
    return value


def eval_poly_derivative(coeffs, t):
    degree = len(coeffs) - 1
    value = 0.0
    for i, c in enumerate(coeffs[:-1]):
        value = value * t + c * (degree - i)
    return value


def eval_position(piece_coeffs, t):
    return (
        eval_poly(piece_coeffs[0], t),
        eval_poly(piece_coeffs[1], t),
        eval_poly(piece_coeffs[2], t),
    )


def eval_velocity(piece_coeffs, t):
    return (
        eval_poly_derivative(piece_coeffs[0], t),
        eval_poly_derivative(piece_coeffs[1], t),
        eval_poly_derivative(piece_coeffs[2], t),
    )


def trajectory_pieces(msg):
    order = int(msg.order_pos)
    coeff_num = order + 1
    if msg.piece_num_pos <= 0 or coeff_num <= 1:
        return []
    if len(msg.time_pos) < msg.piece_num_pos:
        return []
    if len(msg.coef_pos_x) < msg.piece_num_pos * coeff_num:
        return []
    pieces = []
    for i in range(msg.piece_num_pos):
        offset = i * coeff_num
        duration = float(msg.time_pos[i])
        if duration <= 1.0e-6:
            continue
        pieces.append((
            duration,
            (
                [float(v) for v in msg.coef_pos_x[offset:offset + coeff_num]],
                [float(v) for v in msg.coef_pos_y[offset:offset + coeff_num]],
                [float(v) for v in msg.coef_pos_z[offset:offset + coeff_num]],
            ),
        ))
    return pieces


def compute_trajectory_metrics(msg, occupancy, sample_dt, collision_clearance):
    pieces = trajectory_pieces(msg)
    if not pieces:
        return None
    length = 0.0
    duration = sum(piece[0] for piece in pieces)
    max_speed = 0.0
    collision_samples = 0
    sample_count = 0
    last_pos = None
    start_pos = None
    end_pos = None

    for piece_duration, coeffs in pieces:
        steps = max(1, int(math.ceil(piece_duration / sample_dt)))
        for j in range(steps + 1):
            if last_pos is not None and j == 0:
                continue
            t = min(piece_duration, j * piece_duration / steps)
            pos = eval_position(coeffs, t)
            vel = eval_velocity(coeffs, t)
            speed = math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)
            max_speed = max(max_speed, speed)
            if start_pos is None:
                start_pos = pos
            if last_pos is not None:
                length += point_distance(last_pos, pos)
            if occupancy is not None and not occupancy.is_free(pos, collision_clearance):
                collision_samples += 1
            sample_count += 1
            last_pos = pos
            end_pos = pos

    avg_speed = length / duration if duration > 1.0e-6 else 0.0
    return {
        "trajectory_id": int(msg.trajectory_id),
        "piece_num": int(msg.piece_num_pos),
        "duration_s": duration,
        "length_m": length,
        "avg_speed_mps": avg_speed,
        "max_speed_mps": max_speed,
        "collision_samples": collision_samples,
        "sample_count": sample_count,
        "start": start_pos,
        "end": end_pos,
    }


def recent_goal_distance(goal, goal_history, window):
    if not goal_history:
        return float("inf")
    recent = goal_history[-window:] if window > 0 else goal_history
    return min(point_distance(goal, item) for item in recent)


def sample_bounds(args, start, warmup):
    if warmup and args.warmup_radius > 0.0:
        return (
            max(args.x_min, start[0] - args.warmup_radius),
            min(args.x_max, start[0] + args.warmup_radius),
            max(args.y_min, start[1] - args.warmup_radius),
            min(args.y_max, start[1] + args.warmup_radius),
        )
    return args.x_min, args.x_max, args.y_min, args.y_max


def sample_safe_goal(args, rng, occupancy, start, trial_index, goal_history):
    warmup = trial_index <= max(0, args.warmup_trials)
    phase = "warmup" if warmup else "wide"
    required_start_distance = args.min_goal_distance if warmup else max(
        args.min_goal_distance, args.wide_min_goal_distance
    )
    required_recent_distance = 0.0 if warmup else args.min_recent_goal_distance
    x_min, x_max, y_min, y_max = sample_bounds(args, start, warmup)
    candidates = []
    attempts = 0

    for attempt in range(1, args.sample_max_attempts + 1):
        attempts = attempt
        goal = (
            rng.uniform(x_min, x_max),
            rng.uniform(y_min, y_max),
            args.goal_z,
        )
        start_distance = point_distance(goal, start)
        if start_distance < required_start_distance:
            continue
        nearest_recent = recent_goal_distance(goal, goal_history, args.recent_goal_window)
        if nearest_recent < required_recent_distance:
            continue
        clearance = occupancy.clearance(goal, args.goal_clearance)
        if clearance < args.goal_clearance:
            continue
        if warmup:
            return goal, attempt, clearance, phase, required_start_distance, nearest_recent

        spread_score = min(nearest_recent, start_distance) + 0.15 * start_distance
        candidates.append((spread_score, goal, clearance, nearest_recent))
        if len(candidates) >= max(1, args.candidate_batch):
            break

    if candidates:
        _, goal, clearance, nearest_recent = max(candidates, key=lambda item: item[0])
        return goal, attempts, clearance, phase, required_start_distance, nearest_recent

    raise RuntimeError(
        "Failed to sample a collision-free goal. Consider widening bounds or reducing clearance."
    )


def start_demo(demo, log_dir, rviz, fpv_rviz):
    launch_name = DEMO_LAUNCHES[demo]
    log_file = open(log_dir / f"{demo}_roslaunch.log", "w", buffering=1)
    cmd = [
        "roslaunch",
        "task_planner",
        launch_name,
        f"rviz:={str(rviz).lower()}",
        f"fpv_rviz:={str(fpv_rviz).lower()}",
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return proc, log_file


def write_manifest(path, data):
    with open(path, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)


def write_summary(run_dir, summaries):
    csv_path = run_dir / "summary.csv"
    json_path = run_dir / "summary.json"
    fields = [
        "demo",
        "trials",
        "successes",
        "failures",
        "collision_trials",
        "total_replans",
        "mean_first_plan_latency_ms",
        "mean_last_traj_length_m",
        "mean_last_traj_duration_s",
        "mean_last_traj_avg_speed_mps",
        "mean_executed_length_m",
    ]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for item in summaries:
            writer.writerow({k: safe_float(item.get(k)) for k in fields})
    with open(json_path, "w") as f:
        json.dump(summaries, f, indent=2, sort_keys=True)


def mean(values):
    values = [v for v in values if isinstance(v, (int, float)) and not math.isnan(v)]
    return sum(values) / len(values) if values else None


def summarize_demo(demo, rows):
    successes = [r for r in rows if r["success"]]
    return {
        "demo": demo,
        "trials": len(rows),
        "successes": len(successes),
        "failures": len(rows) - len(successes),
        "collision_trials": sum(1 for r in rows if r["collision"]),
        "total_replans": sum(r["replan_count"] for r in rows),
        "mean_first_plan_latency_ms": mean([r["first_plan_latency_ms"] for r in successes]),
        "mean_last_traj_length_m": mean([r["last_traj_length_m"] for r in successes]),
        "mean_last_traj_duration_s": mean([r["last_traj_duration_s"] for r in successes]),
        "mean_last_traj_avg_speed_mps": mean([r["last_traj_avg_speed_mps"] for r in successes]),
        "mean_executed_length_m": mean([r["executed_length_m"] for r in successes]),
    }


def write_row(writer, csv_file, row):
    writer.writerow({key: safe_float(row.get(key)) for key in CSV_FIELDS})
    csv_file.flush()


def run_trial(node, args, rng, occupancy, demo, trial_index, goal_history):
    state = node.get_odom_state()
    if state is None:
        raise RuntimeError("No odometry available.")
    start_pos, _ = state
    goal, attempts, clearance, phase, required_distance, nearest_recent = sample_safe_goal(
        args, rng, occupancy, start_pos, trial_index, goal_history
    )

    node.reset_trial_events()
    start_wall = time.monotonic()
    node.publish_goal(goal, start_pos)

    first_event = None
    deadline = time.monotonic() + args.plan_timeout
    seen_ids = set()
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        events = [e for e in node.copy_traj_events() if e.wall_time >= start_wall]
        for event in events:
            traj_id = int(event.msg.trajectory_id)
            if traj_id in seen_ids:
                continue
            seen_ids.add(traj_id)
            first_event = event
            break
        if first_event is not None:
            break
        time.sleep(0.02)

    row = {
        "demo": demo,
        "trial": trial_index,
        "success": False,
        "failure_reason": "",
        "goal_x": goal[0],
        "goal_y": goal[1],
        "goal_z": goal[2],
        "start_x": start_pos[0],
        "start_y": start_pos[1],
        "start_z": start_pos[2],
        "goal_distance_m": point_distance(goal, start_pos),
        "sampling_phase": phase,
        "required_goal_distance_m": required_distance,
        "nearest_recent_goal_distance_m": nearest_recent if math.isfinite(nearest_recent) else None,
        "sample_attempts": attempts,
        "goal_clearance_m": clearance,
        "first_plan_latency_ms": None,
        "elapsed_s": None,
        "trajectory_count": 0,
        "replan_count": 0,
        "first_traj_id": None,
        "last_traj_id": None,
        "first_traj_duration_s": None,
        "first_traj_length_m": None,
        "first_traj_avg_speed_mps": None,
        "first_traj_max_speed_mps": None,
        "last_traj_duration_s": None,
        "last_traj_length_m": None,
        "last_traj_avg_speed_mps": None,
        "last_traj_max_speed_mps": None,
        "executed_length_m": 0.0,
        "executed_avg_speed_mps": None,
        "planned_collision_traj_count": 0,
        "planned_collision_sample_count": 0,
        "executed_collision_sample_count": 0,
        "collision": False,
    }

    if first_event is None:
        row["failure_reason"] = "plan_timeout"
        row["elapsed_s"] = time.monotonic() - start_wall
        return row

    row["first_plan_latency_ms"] = (first_event.wall_time - start_wall) * 1000.0
    finish_deadline = time.monotonic() + args.trial_timeout
    last_exec_pos = start_pos
    last_odom_sample = 0.0
    reached_time = None
    finish_reason = "trial_timeout"

    while not rospy.is_shutdown() and time.monotonic() < finish_deadline:
        now = time.monotonic()
        state = node.get_odom_state()
        if state is not None and now - last_odom_sample >= args.odom_sample_dt:
            pos, speed = state
            row["executed_length_m"] += point_distance(last_exec_pos, pos)
            if not occupancy.is_free(pos, args.collision_clearance):
                row["executed_collision_sample_count"] += 1
            last_exec_pos = pos
            last_odom_sample = now
            if point_distance(pos, goal) <= args.goal_tolerance and speed <= args.speed_tolerance:
                if reached_time is None:
                    reached_time = now
                elif now - reached_time >= args.settle_time:
                    finish_reason = ""
                    break
            else:
                reached_time = None
        time.sleep(0.02)

    elapsed = time.monotonic() - start_wall
    row["elapsed_s"] = elapsed
    row["executed_avg_speed_mps"] = row["executed_length_m"] / elapsed if elapsed > 1.0e-6 else 0.0

    unique_events = []
    seen_ids = set()
    for event in node.copy_traj_events():
        if event.wall_time < start_wall:
            continue
        traj_id = int(event.msg.trajectory_id)
        if traj_id in seen_ids:
            continue
        seen_ids.add(traj_id)
        unique_events.append(event)

    metrics = []
    for event in unique_events:
        item = compute_trajectory_metrics(
            event.msg, occupancy, args.trajectory_sample_dt, args.collision_clearance
        )
        if item is not None:
            metrics.append(item)

    if metrics:
        first = metrics[0]
        last = metrics[-1]
        row["trajectory_count"] = len(metrics)
        row["replan_count"] = max(0, len(metrics) - 1)
        row["first_traj_id"] = first["trajectory_id"]
        row["last_traj_id"] = last["trajectory_id"]
        row["first_traj_duration_s"] = first["duration_s"]
        row["first_traj_length_m"] = first["length_m"]
        row["first_traj_avg_speed_mps"] = first["avg_speed_mps"]
        row["first_traj_max_speed_mps"] = first["max_speed_mps"]
        row["last_traj_duration_s"] = last["duration_s"]
        row["last_traj_length_m"] = last["length_m"]
        row["last_traj_avg_speed_mps"] = last["avg_speed_mps"]
        row["last_traj_max_speed_mps"] = last["max_speed_mps"]
        row["planned_collision_traj_count"] = sum(1 for item in metrics if item["collision_samples"] > 0)
        row["planned_collision_sample_count"] = sum(item["collision_samples"] for item in metrics)

    row["collision"] = (
        row["planned_collision_traj_count"] > 0
        or row["planned_collision_sample_count"] > 0
        or row["executed_collision_sample_count"] > 0
    )
    row["success"] = finish_reason == "" and not row["collision"]
    row["failure_reason"] = finish_reason if not row["success"] else ""
    if row["collision"] and not row["failure_reason"]:
        row["failure_reason"] = "collision"
    return row


def run_demo(node, args, rng, demo, run_dir):
    rospy.loginfo("Starting benchmark demo '%s'.", demo)
    previous_cloud_seq = node.current_cloud_seq()
    previous_odom_seq = node.current_odom_seq()
    proc, launch_log = start_demo(demo, run_dir, args.rviz, args.fpv_rviz)
    rows = []
    csv_path = run_dir / f"{demo}.csv"
    try:
        if not node.wait_for_ready(previous_cloud_seq, previous_odom_seq):
            raise RuntimeError(f"Demo '{demo}' did not become ready.")
        occupancy = OccupancyIndex(node.latest_cloud_msg(), args.map_voxel)
        rospy.loginfo(
            "Global map for %s: %d points, %d occupied voxels, bounds=%s",
            demo,
            occupancy.point_count,
            len(occupancy.occupied),
            [round(v, 3) for v in occupancy.bounds],
        )
        if args.startup_wait > 0.0:
            rospy.loginfo("Waiting %.2f s for planner local map warmup.", args.startup_wait)
            rospy.sleep(args.startup_wait)
        goal_history = []
        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for trial in range(1, args.trials + 1):
                if rospy.is_shutdown():
                    break
                row = run_trial(node, args, rng, occupancy, demo, trial, goal_history)
                rows.append(row)
                goal_history.append((row["goal_x"], row["goal_y"], row["goal_z"]))
                write_row(writer, csv_file, row)
                rospy.loginfo(
                    "[%s %d/%d] success=%s phase=%s replans=%s length=%.3f duration=%.3f collision=%s",
                    demo,
                    trial,
                    args.trials,
                    row["success"],
                    row["sampling_phase"],
                    row["replan_count"],
                    row["last_traj_length_m"] or 0.0,
                    row["last_traj_duration_s"] or 0.0,
                    row["collision"],
                )
    finally:
        stop_process_group(proc, f"roslaunch {demo}")
        launch_log.close()
        time.sleep(1.0)
    return rows


def main():
    args = parse_args()
    demos = expand_demos(args.demo)
    random_seed = args.seed
    rng = random.Random(random_seed)

    roscore_proc = ensure_ros_master()
    rospy.init_node("click_state2state_benchmark", anonymous=False)

    pkg_path = Path(rospkg.RosPack().get_path("general_planner"))
    base_log_dir = Path(args.log_dir) if args.log_dir else pkg_path / "log"
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    run_dir = base_log_dir / f"click_state2state_benchmark_{stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "stamp": stamp,
        "demos": demos,
        "trials_per_demo": args.trials,
        "seed": random_seed,
        "sampling_bounds": {
            "x": [args.x_min, args.x_max],
            "y": [args.y_min, args.y_max],
            "z": args.goal_z,
        },
        "sampling_strategy": {
            "warmup_trials": args.warmup_trials,
            "warmup_radius": args.warmup_radius,
            "wide_min_goal_distance": args.wide_min_goal_distance,
            "min_recent_goal_distance": args.min_recent_goal_distance,
            "recent_goal_window": args.recent_goal_window,
            "candidate_batch": args.candidate_batch,
        },
        "goal_clearance": args.goal_clearance,
        "collision_clearance": args.collision_clearance,
        "trajectory_sample_dt": args.trajectory_sample_dt,
        "rviz": args.rviz,
        "fpv_rviz": args.fpv_rviz,
        "log_dir": str(run_dir),
    }
    write_manifest(run_dir / "manifest.json", manifest)
    rospy.loginfo("Benchmark logs will be written to %s", run_dir)

    node = BenchmarkNode(args)
    summaries = []
    try:
        for demo in demos:
            rows = run_demo(node, args, rng, demo, run_dir)
            summaries.append(summarize_demo(demo, rows))
            write_summary(run_dir, summaries)
    finally:
        if roscore_proc is not None:
            stop_process_group(roscore_proc, "roscore")

    write_summary(run_dir, summaries)
    rospy.loginfo("Benchmark finished. Logs: %s", run_dir)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
