#!/usr/bin/env python3
"""Closed-loop A/B for dense vs fixed hull vs two-stage hull.

Uses the historical goal [69.032, 1.901, 1.500] and click_real_highspeed base
config. Fast LBFGS stays enabled in all three arms so the comparison isolates
the cost representation / adaptive topology.
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


GOAL = (69.032, 1.901, 1.500)
BASE_CONFIG_NAME = "click_real_highspeed.yaml"

VARIANTS = (
    {
        "label": "fast_dense",
        "expected_mode": "dense",
        "overrides": {
            "convex_hull_en": "false",
            "convex_hull_adaptive_en": "false",
            "convex_hull_alm_en": "false",
            "lbfgs_fast_en": "true",
        },
    },
    {
        "label": "fast_hull_fixed_d2",
        "expected_mode": "convex_bezier_v2_d2",
        "overrides": {
            "convex_hull_en": "true",
            "convex_hull_adaptive_en": "false",
            "convex_hull_alm_en": "false",
            "convex_hull_subdivision_depth": "2",
            "convex_hull_cost_version": "2",
            "lbfgs_fast_en": "true",
        },
    },
    {
        "label": "fast_hull_twostage_d2",
        "expected_mode": "convex_bezier_v2_twostage_d2",
        "overrides": {
            "convex_hull_en": "true",
            "convex_hull_adaptive_en": "true",
            "convex_hull_alm_en": "false",
            "convex_hull_subdivision_depth": "2",
            "convex_hull_cost_version": "2",
            "lbfgs_fast_en": "true",
        },
    },
)


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def write_variant_config(base: Path, out: Path, overrides: dict) -> None:
    lines = base.read_text().splitlines()
    keys = set(overrides)
    rewritten = []
    seen = set()
    for line in lines:
        stripped = line.lstrip()
        matched = None
        for key in keys:
            if stripped.startswith(f"{key}:"):
                matched = key
                break
        if matched is None:
            rewritten.append(line)
            continue
        indent = line[: len(line) - len(stripped)]
        rewritten.append(f"{indent}{matched}: {overrides[matched]}")
        seen.add(matched)
    missing = keys - seen
    if missing:
        raise RuntimeError(f"Missing keys in base config: {sorted(missing)}")
    out.write_text("\n".join(rewritten) + "\n")


def run(cmd, **kwargs):
    print("+", " ".join(cmd), flush=True)
    return subprocess.Popen(cmd, **kwargs)


def stop_process_group(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=5)


def wait_for_topics(timeout_s: float = 60.0) -> None:
    deadline = time.time() + timeout_s
    # click_real_highspeed uses lidar_slam odom rather than /odom.
    needed = {"/lidar_slam/odom", "/cloud_registered", "/goal"}
    last = set()
    while time.time() < deadline:
        try:
            out = subprocess.check_output(
                ["rostopic", "list"], text=True, stderr=subprocess.DEVNULL
            )
            topics = set(out.split())
            last = topics
            if needed.issubset(topics):
                print(f"Topics ready: {sorted(needed)}", flush=True)
                return
        except subprocess.CalledProcessError:
            pass
        time.sleep(1.0)
    raise RuntimeError(
        f"Timed out waiting for topics {sorted(needed)}; saw={sorted(last)}"
    )


def publish_goal(goal, duration_s: float = 1.5) -> None:
    x, y, z = goal
    # Prefer a one-shot latched-style burst via rospy for reliability under
    # redirected stdout / process groups.
    code = f"""
import time
import rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node('hull_ab_goal_pub', anonymous=True)
pub = rospy.Publisher('/goal', PoseStamped, queue_size=1)
msg = PoseStamped()
msg.header.frame_id = 'world'
msg.pose.position.x = {x}
msg.pose.position.y = {y}
msg.pose.position.z = {z}
msg.pose.orientation.w = 1.0
rate = rospy.Rate(20)
end = time.time() + {duration_s}
while not rospy.is_shutdown() and time.time() < end:
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()
"""
    proc = run(["python3", "-c", code], preexec_fn=os.setsid)
    try:
        proc.wait(timeout=duration_s + 10)
    except subprocess.TimeoutExpired:
        stop_process_group(proc)
    print(f"Published goal {goal} for {duration_s:.1f}s", flush=True)


def summarize_csv(path: Path) -> dict:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"Empty timing CSV: {path}")

    def nums(key: str) -> list[float]:
        values = []
        for row in rows:
            raw = (row.get(key) or "").strip()
            if raw == "":
                continue
            values.append(float(raw))
        return values

    def total_ms(key: str) -> float:
        return 1000.0 * sum(nums(key))

    def mean(key: str) -> float:
        values = nums(key)
        return sum(values) / len(values) if values else float("nan")

    mode = (rows[-1].get("EXP_COST_MODE") or "").strip()
    evals = nums("EXP_EVALUATIONS")
    iters = nums("EXP_LBFGS_ITERATIONS")
    return {
        "rows": len(rows),
        "mode": mode,
        "path_length_proxy_opt_calls": len(rows),
        "sum_exp_traj_opt_ms": total_ms("EXP_TRAJ_OPT"),
        "sum_total_replan_ms": total_ms("TOTAL_REPLAN"),
        "sum_lbfgs_ms": sum(nums("EXP_LBFGS_MS")),
        "sum_minco_eval_ms": sum(nums("EXP_MINCO_EVALUATION_MS")),
        "sum_dense_integral_ms": sum(nums("EXP_DENSE_INTEGRAL_MS")),
        "sum_control_point_ms": sum(nums("EXP_CONTROL_POINT_FUNCTIONAL_MS")),
        "mean_evals": mean("EXP_EVALUATIONS"),
        "mean_iters": mean("EXP_LBFGS_ITERATIONS"),
        "mean_line_search_per_iter": mean("EXP_AVG_LINE_SEARCH_EVALS"),
        "mean_hull_checks": mean("EXP_HULL_CONTROL_CHECKS_PER_EVAL"),
        "mean_coarse": mean("EXP_ADAPTIVE_COARSE_SEGMENTS"),
        "mean_fine": mean("EXP_ADAPTIVE_FINE_SEGMENTS"),
        "mean_opt_ms": 1000.0 * mean("EXP_TRAJ_OPT"),
        "total_evaluations": sum(evals),
        "total_iterations": sum(iters),
    }


def print_summary(results: list[tuple[str, dict]]) -> None:
    print("\n=== Closed-loop comparison (goal [69.032, 1.901, 1.500]) ===")
    header = (
        f"{'variant':24s} {'mode':32s} {'calls':>5s} {'evals':>8s} "
        f"{'iters':>8s} {'opt_ms':>10s} {'lbfgs_ms':>10s} "
        f"{'minco_ms':>10s} {'dense_ms':>10s} {'ctrl_ms':>10s} "
        f"{'evals/call':>10s} {'hull_chk':>8s} {'coarse':>6s} {'fine':>6s}"
    )
    print(header)
    baseline = results[0][1]
    for label, stats in results:
        print(
            f"{label:24s} {stats['mode']:32s} {stats['rows']:5d} "
            f"{stats['total_evaluations']:8.0f} {stats['total_iterations']:8.0f} "
            f"{stats['sum_exp_traj_opt_ms']:10.2f} {stats['sum_lbfgs_ms']:10.2f} "
            f"{stats['sum_minco_eval_ms']:10.2f} {stats['sum_dense_integral_ms']:10.2f} "
            f"{stats['sum_control_point_ms']:10.2f} {stats['mean_evals']:10.2f} "
            f"{stats['mean_hull_checks']:8.1f} {stats['mean_coarse']:6.2f} "
            f"{stats['mean_fine']:6.2f}"
        )
    print("\nRelative to fast_dense:")
    for label, stats in results[1:]:
        def ratio(key: str) -> str:
            b = baseline[key]
            v = stats[key]
            if b == 0:
                return "n/a"
            return f"{v / b:.3f}x ({(v - b) / b * 100:+.1f}%)"

        print(
            f"  {label}: opt={ratio('sum_exp_traj_opt_ms')}, "
            f"lbfgs={ratio('sum_lbfgs_ms')}, "
            f"evals={ratio('total_evaluations')}, "
            f"minco={ratio('sum_minco_eval_ms')}"
        )
    if len(results) >= 3:
        fixed = results[1][1]
        twostage = results[2][1]
        print("\nRelative to fast_hull_fixed_d2:")

        def ratio2(key: str) -> str:
            b = fixed[key]
            v = twostage[key]
            if b == 0:
                return "n/a"
            return f"{v / b:.3f}x ({(v - b) / b * 100:+.1f}%)"

        print(
            f"  fast_hull_twostage_d2: opt={ratio2('sum_exp_traj_opt_ms')}, "
            f"lbfgs={ratio2('sum_lbfgs_ms')}, "
            f"evals={ratio2('total_evaluations')}, "
            f"ctrl={ratio2('sum_control_point_ms')}, "
            f"hull_checks={ratio2('mean_hull_checks')}"
        )


def run_one(variant: dict, args) -> tuple[str, dict, Path]:
    root = package_root()
    log_dir = root / "log"
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cfg_path = out_dir / f"{variant['label']}.yaml"
    write_variant_config(root / "config" / BASE_CONFIG_NAME, cfg_path, variant["overrides"])

    # Clear previous mode-named CSV so we do not mix runs.
    for old in log_dir.glob("time_consuming_*.csv"):
        if old.name.startswith("time_consuming_"):
            # Keep historical archives; only remove exact expected mode file.
            pass
    expected_csv = log_dir / f"time_consuming_{variant['expected_mode']}.csv"
    if expected_csv.exists():
        expected_csv.unlink()

    env = os.environ.copy()
    env.setdefault("ROS_MASTER_URI", "http://localhost:11311")
    env.setdefault("ROS_HOSTNAME", "localhost")

    roscore_log = out_dir / f"{variant['label']}_roscore.log"
    launch_log = out_dir / f"{variant['label']}_launch.log"
    roscore_log_f = roscore_log.open("w")
    launch_log_f = launch_log.open("w")
    roscore = run(
        ["roscore"],
        preexec_fn=os.setsid,
        env=env,
        stdout=roscore_log_f,
        stderr=subprocess.STDOUT,
    )
    time.sleep(2.0)
    launch = run(
        [
            "roslaunch",
            "task_planner",
            "click_demo.launch",
            f"planner_config_path:={cfg_path}",
            "rviz:=false",
            "fpv_rviz:=false",
        ],
        preexec_fn=os.setsid,
        env=env,
        stdout=launch_log_f,
        stderr=subprocess.STDOUT,
    )
    try:
        wait_for_topics(timeout_s=90.0)
        time.sleep(args.startup_wait)
        publish_goal(GOAL, duration_s=args.goal_publish_time)
        print(f"[{variant['label']}] flying for {args.duration:.1f}s ...", flush=True)
        time.sleep(args.duration)
    finally:
        stop_process_group(launch)
        stop_process_group(roscore)
        # Ensure leftover ROS nodes die between variants.
        subprocess.call(["pkill", "-9", "-f", "fsm_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(["pkill", "-9", "-f", "perfect_drone"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(["pkill", "-9", "-f", "rosmaster"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(["pkill", "-9", "-f", "roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for handle in (locals().get("roscore_log_f"), locals().get("launch_log_f")):
            if handle is not None:
                handle.close()
        time.sleep(2.0)

    if not expected_csv.exists():
        # Fallback: newest time_consuming_*.csv
        candidates = sorted(log_dir.glob("time_consuming_*.csv"), key=lambda p: p.stat().st_mtime)
        if not candidates:
            raise RuntimeError(f"No timing CSV produced for {variant['label']}")
        expected_csv = candidates[-1]
        print(f"[{variant['label']}] warning: using fallback CSV {expected_csv.name}", flush=True)

    archived = out_dir / f"time_consuming_{variant['label']}.csv"
    archived.write_bytes(expected_csv.read_bytes())
    stats = summarize_csv(archived)
    if stats["mode"] and stats["mode"] != variant["expected_mode"]:
        print(
            f"[{variant['label']}] warning: mode={stats['mode']} "
            f"expected={variant['expected_mode']}",
            flush=True,
        )
    return variant["label"], stats, archived


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--startup-wait", type=float, default=3.0)
    parser.add_argument("--goal-publish-time", type=float, default=1.5)
    parser.add_argument(
        "--out-dir",
        default=str(
            package_root()
            / "log"
            / f"hull_twostage_ab_{time.strftime('%Y%m%d_%H%M%S')}"
        ),
    )
    parser.add_argument(
        "--only",
        choices=[v["label"] for v in VARIANTS],
        action="append",
        default=None,
    )
    args = parser.parse_args()

    selected = [
        v for v in VARIANTS if args.only is None or v["label"] in args.only
    ]
    results = []
    for variant in selected:
        print(f"\n===== Running {variant['label']} =====", flush=True)
        label, stats, path = run_one(variant, args)
        print(f"[{label}] archived {path}", flush=True)
        print(f"[{label}] stats={stats}", flush=True)
        results.append((label, stats))

    print_summary(results)
    summary_path = Path(args.out_dir) / "summary.csv"
    with summary_path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["label"] + list(results[0][1].keys()),
        )
        writer.writeheader()
        for label, stats in results:
            row = {"label": label}
            row.update(stats)
            writer.writerow(row)
    print(f"\nWrote {summary_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
