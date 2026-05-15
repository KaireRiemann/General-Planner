#!/usr/bin/env python3

import argparse
import csv
import json
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rospkg


DEMO_CONFIGS = {
    "click": "click_smooth_ros1.yaml",
    "smooth": "click_smooth_ros1.yaml",
    "esdf": "click_esdf_ros1.yaml",
    "plain": "click_plain_ros1.yaml",
}

SUMMARY_FIELDS = [
    "solver",
    "demo",
    "status",
    "trials",
    "successes",
    "failures",
    "success_rate",
    "collision_trials",
    "collision_rate",
    "emergency_stop_count",
    "total_replans",
    "replan_deadline_ms",
    "total_deadline_miss_count",
    "deadline_check_count",
    "deadline_miss_rate",
    "mean_replan_cycle_time_ms",
    "max_replan_cycle_time_ms",
    "mean_replan_interval_ms",
    "mean_planning_frequency_hz",
    "total_flight_time_s",
    "mean_flight_time_s",
    "mean_trajectory_duration_s",
    "total_length_m",
    "mean_corridor_generation_time_ms",
    "mean_corridor_time_ms",
    "mean_trajectory_optimization_time_ms",
    "mean_exp_frontend_time_ms",
    "mean_exp_opt_time_ms",
    "mean_backup_frontend_time_ms",
    "mean_backup_opt_time_ms",
    "mean_total_replan_time_ms",
    "mean_optimization_time_ms",
    "mean_speed_mps",
    "child_log_dir",
    "config_path",
    "cases_path",
]


def str_to_bool(value):
    if isinstance(value, bool):
        return value
    value = value.lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Expected a boolean value, got '{value}'.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run corridor benchmark variants with different ellipsoid optimizers."
    )
    parser.add_argument("--demo", default="click")
    parser.add_argument("--solvers", default="classic,hom,hom_no_normalization,hom_fallback")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--cases-path", default="")
    parser.add_argument("--x-min", type=float, default=-40.0)
    parser.add_argument("--x-max", type=float, default=40.0)
    parser.add_argument("--y-min", type=float, default=-40.0)
    parser.add_argument("--y-max", type=float, default=40.0)
    parser.add_argument("--goal-z", type=float, default=1.5)
    parser.add_argument("--min-goal-distance", type=float, default=4.0)
    parser.add_argument("--warmup-trials", type=int, default=0)
    parser.add_argument("--warmup-radius", type=float, default=4.0)
    parser.add_argument("--wide-min-goal-distance", type=float, default=15.0)
    parser.add_argument("--min-recent-goal-distance", type=float, default=10.0)
    parser.add_argument("--recent-goal-window", type=int, default=12)
    parser.add_argument("--candidate-batch", type=int, default=32)
    parser.add_argument("--goal-clearance", type=float, default=0.45)
    parser.add_argument("--collision-clearance", type=float, default=0.25)
    parser.add_argument("--plan-timeout", type=float, default=8.0)
    parser.add_argument("--trial-timeout", type=float, default=60.0)
    parser.add_argument("--startup-wait", type=float, default=2.0)
    parser.add_argument("--goal-publish-time", type=float, default=1.0)
    parser.add_argument("--goal-publish-rate", type=float, default=10.0)
    parser.add_argument("--manual-goals", type=str_to_bool, default=False)
    parser.add_argument("--manual-goal-topic", default="/goal")
    parser.add_argument("--manual-goal-timeout", type=float, default=0.0)
    parser.add_argument("--rviz", type=str_to_bool, default=False)
    parser.add_argument("--fpv-rviz", type=str_to_bool, default=False)
    args, _ = parser.parse_known_args()
    if args.cases_path in ("", "__auto__", "auto"):
        args.cases_path = ""
    return args


def split_csv_arg(value):
    return [item.strip() for item in value.split(",") if item.strip()]


def expand_demos(value):
    demos = split_csv_arg(value)
    if not demos or demos == ["all"]:
        return ["click"]
    out = []
    for demo in demos:
        demo = demo.lower()
        if demo not in DEMO_CONFIGS:
            raise ValueError(f"Unsupported demo '{demo}'. Use click, esdf, plain, smooth, or all.")
        demo = "click" if demo == "smooth" else demo
        if demo not in out:
            out.append(demo)
    return out


def solver_file_token(solver):
    token = re.sub(r"[^A-Za-z0-9_]+", "_", solver.strip().lower())
    return token or "solver"


def write_solver_config(src_path, dst_path, solver):
    lines = src_path.read_text().splitlines()
    solver_written = False
    fallback_written = False
    iris_insert_index = None
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("ellipsoid_optimizer:"):
            out.append(f"  ellipsoid_optimizer: {solver}")
            solver_written = True
        elif stripped.startswith("ellipsoid_optimizer_fallback:"):
            out.append("  ellipsoid_optimizer_fallback: false")
            fallback_written = True
        else:
            out.append(line)
            if stripped.startswith("iris_iter_num:"):
                iris_insert_index = len(out)
    if not solver_written or not fallback_written:
        insert_at = iris_insert_index if iris_insert_index is not None else len(out)
        insert_lines = []
        if not solver_written:
            insert_lines.append(f"  ellipsoid_optimizer: {solver}")
        if not fallback_written:
            insert_lines.append("  ellipsoid_optimizer_fallback: false")
        out[insert_at:insert_at] = insert_lines
    dst_path.write_text("\n".join(out) + "\n")


def latest_child_run_dir(base_dir):
    candidates = [p for p in base_dir.glob("click_state2state_benchmark_*") if p.is_dir()]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def safe_float(value):
    if isinstance(value, float):
        return f"{value:.6f}"
    return value if value is not None else ""


def write_summary(run_dir, rows):
    csv_path = run_dir / "summary.csv"
    json_path = run_dir / "summary.json"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: safe_float(row.get(key)) for key in SUMMARY_FIELDS})
    with open(json_path, "w") as f:
        json.dump(rows, f, indent=2, sort_keys=True)


def stop_process_group(proc):
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
    except Exception:
        pass


def benchmark_common_cmd(args, demo, child_log_base):
    return [
        "rosrun",
        "general_planner",
        "click_state2state_benchmark.py",
        "--demo", demo,
        "--trials", str(args.trials),
        "--seed", str(args.seed),
        "--log-dir", str(child_log_base),
        "--metric-set", "state",
        "--plan-only", "false",
        "--restart-per-trial", "false",
        "--x-min", str(args.x_min),
        "--x-max", str(args.x_max),
        "--y-min", str(args.y_min),
        "--y-max", str(args.y_max),
        "--goal-z", str(args.goal_z),
        "--min-goal-distance", str(args.min_goal_distance),
        "--warmup-trials", str(args.warmup_trials),
        "--warmup-radius", str(args.warmup_radius),
        "--wide-min-goal-distance", str(args.wide_min_goal_distance),
        "--min-recent-goal-distance", str(args.min_recent_goal_distance),
        "--recent-goal-window", str(args.recent_goal_window),
        "--candidate-batch", str(args.candidate_batch),
        "--goal-clearance", str(args.goal_clearance),
        "--collision-clearance", str(args.collision_clearance),
        "--plan-timeout", str(args.plan_timeout),
        "--trial-timeout", str(args.trial_timeout),
        "--startup-wait", str(args.startup_wait),
        "--goal-publish-time", str(args.goal_publish_time),
        "--goal-publish-rate", str(args.goal_publish_rate),
        "--manual-goals", str(args.manual_goals).lower(),
        "--manual-goal-topic", args.manual_goal_topic,
        "--manual-goal-timeout", str(args.manual_goal_timeout),
        "--rviz", str(args.rviz).lower(),
        "--fpv-rviz", str(args.fpv_rviz).lower(),
    ]


def run_child(cmd, log_path):
    with open(log_path, "w", buffering=1) as log_file:
        proc = subprocess.Popen(cmd, stdout=log_file, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        try:
            ret = proc.wait()
        except KeyboardInterrupt:
            stop_process_group(proc)
            raise
    return ret


def generate_cases(args, run_dir, demo):
    case_log_base = run_dir / "case_sampling" / demo
    case_log_base.mkdir(parents=True, exist_ok=True)
    cases_path = Path(args.cases_path) if args.cases_path else run_dir / f"{demo}_cases.csv"
    subprocess_log_path = run_dir / f"case_sampling_{demo}.log"
    cmd = benchmark_common_cmd(args, demo, case_log_base)
    cmd.extend([
        "--sample-cases-only", "true",
        "--write-cases-path", str(cases_path),
    ])
    print(f"[corridor_benchmark] sampling fixed goal cases for demo={demo}")
    ret = run_child(cmd, subprocess_log_path)
    if ret != 0:
        raise RuntimeError(f"Case sampling failed for demo={demo}, see {subprocess_log_path}")
    return cases_path


def run_one(args, run_dir, config_dir, planner_config_dir, demo, solver, cases_path):
    base_config = DEMO_CONFIGS[demo]
    solver_token = solver_file_token(solver)
    config_path = config_dir / f"{demo}_{solver_token}_{base_config}"
    write_solver_config(planner_config_dir / base_config, config_path, solver)

    child_log_base = run_dir / solver_token / demo
    child_log_base.mkdir(parents=True, exist_ok=True)
    subprocess_log_path = run_dir / f"{solver_token}_{demo}_benchmark.log"

    cmd = benchmark_common_cmd(args, demo, child_log_base)
    cmd.extend([
        "--planner-config-path", str(config_path),
        "--ellipsoid-optimizer-label", solver,
        "--cases-path", str(cases_path),
    ])

    print(f"[corridor_benchmark] running solver={solver} demo={demo} cases={cases_path}")
    ret = run_child(cmd, subprocess_log_path)

    child_run_dir = latest_child_run_dir(child_log_base)
    row = {
        "solver": solver,
        "demo": demo,
        "status": "ok" if ret == 0 else f"failed({ret})",
        "child_log_dir": str(child_run_dir) if child_run_dir is not None else "",
        "config_path": str(config_path),
        "cases_path": str(cases_path),
    }
    if child_run_dir is not None and (child_run_dir / "summary.json").exists():
        with open(child_run_dir / "summary.json") as f:
            summaries = json.load(f)
        if summaries:
            row.update(summaries[0])
            row["solver"] = solver
            row["status"] = "ok" if ret == 0 else f"failed({ret})"
    return row


def main():
    args = parse_args()
    demos = expand_demos(args.demo)
    solvers = split_csv_arg(args.solvers)
    if not solvers:
        raise RuntimeError("No solvers requested.")

    pkg_path = Path(rospkg.RosPack().get_path("general_planner"))
    repo_root = pkg_path.parent
    planner_config_dir = pkg_path / "config"
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    base_log_dir = Path(args.log_dir) if args.log_dir else pkg_path / "log" / "corridor_log"
    run_dir = base_log_dir / stamp
    config_dir = run_dir / "configs"
    config_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "stamp": stamp,
        "repo_root": str(repo_root),
        "run_dir": str(run_dir),
        "demos": demos,
        "solvers": solvers,
        "cases_per_solver_demo": args.trials,
        "seed": args.seed,
        "cases_path_arg": args.cases_path,
        "note": "Fixed goal cases are sampled once. Each ellipsoid optimizer starts from the default demo initial state and flies the same goal sequence continuously.",
    }
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

    rows = []
    failed = False
    try:
        case_paths = {}
        for demo in demos:
            if args.cases_path:
                case_paths[demo] = Path(args.cases_path)
            else:
                case_paths[demo] = generate_cases(args, run_dir, demo)
        manifest["case_paths"] = {demo: str(path) for demo, path in case_paths.items()}
        (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

        for demo in demos:
            for solver in solvers:
                row = run_one(args, run_dir, config_dir, planner_config_dir, demo, solver, case_paths[demo])
                rows.append(row)
                failed = failed or row.get("status") != "ok"
                write_summary(run_dir, rows)
                time.sleep(1.0)
    finally:
        write_summary(run_dir, rows)
        print(f"[corridor_benchmark] logs: {run_dir}")

    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
