#!/usr/bin/env bash
# Same-goal A/B: dense vs stable depth-2 Bezier hull, 5 repeats each.
set -eo pipefail

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/devel/setup.bash
set -u

REPEATS=${REPEATS:-5}
FLY_SEC=${FLY_SEC:-40}
GOAL_X=${GOAL_X:-69.032}
GOAL_Y=${GOAL_Y:-1.901}
GOAL_Z=${GOAL_Z:-1.500}

CFG=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/config/click_real_highspeed.yaml
LOGDIR=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log
OUT=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log/dense_vs_hull_r5_$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
cp -a "$CFG" "$OUT/click_real_highspeed.yaml.bak"

set_yaml_key() {
  local key="$1" value="$2"
  python3 - "$CFG" "$key" "$value" <<'PY'
import sys
from pathlib import Path
path, key, value = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
lines = path.read_text().splitlines()
out, seen = [], False
for line in lines:
    s = line.lstrip()
    if s.startswith(key + ":"):
        indent = line[:len(line)-len(s)]
        out.append(f"{indent}{key}: {value}")
        seen = True
    else:
        out.append(line)
if not seen:
    raise SystemExit(f"missing key {key}")
path.write_text("\n".join(out) + "\n")
PY
}

stop_stack() {
  kill "$(cat "$OUT/roscore.pid" 2>/dev/null)" 2>/dev/null || true
  kill "$(cat "$OUT/launch.pid" 2>/dev/null)" 2>/dev/null || true
  sleep 2
  kill -9 "$(cat "$OUT/roscore.pid" 2>/dev/null)" 2>/dev/null || true
  kill -9 "$(cat "$OUT/launch.pid" 2>/dev/null)" 2>/dev/null || true
  killall -q fsm_node perfect_drone rosmaster rosout 2>/dev/null || true
  sleep 2
}

configure_route() {
  local hull="$1"
  set_yaml_key convex_hull_en "$hull"
  set_yaml_key convex_hull_basis 0
  set_yaml_key convex_hull_flatness_en true
  set_yaml_key convex_hull_require_certification false
  set_yaml_key lbfgs_fast_en true
  set_yaml_key lbfgs_fast_phase0_guards_en false
  grep -E "convex_hull_en:|convex_hull_basis:|convex_hull_flatness_en:|convex_hull_require_certification:|lbfgs_fast_en:" "$CFG"
}

run_one() {
  local label="$1"
  echo "===== $label ====="
  find "$LOGDIR" -maxdepth 1 -name 'time_consuming_*.csv' -delete 2>/dev/null || true

  roscore >"$OUT/${label}_roscore.log" 2>&1 &
  echo $! >"$OUT/roscore.pid"
  sleep 2
  roslaunch task_planner click_demo.launch rviz:=false fpv_rviz:=false \
    >"$OUT/${label}_launch.log" 2>&1 &
  echo $! >"$OUT/launch.pid"

  for i in $(seq 1 60); do
    if rostopic list 2>/dev/null | grep -q "/lidar_slam/odom"; then
      break
    fi
    sleep 1
  done
  sleep 3

  echo "publish goal ($GOAL_X, $GOAL_Y, $GOAL_Z)"
  timeout 2 rostopic pub /goal geometry_msgs/PoseStamped \
    "{header: {frame_id: \"world\"}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {w: 1.0}}}" \
    -r 20 >/dev/null 2>&1 || true

  echo "fly ${FLY_SEC}s"
  sleep "$FLY_SEC"
  stop_stack

  local csv
  csv=$(ls -t "$LOGDIR"/time_consuming_*.csv 2>/dev/null | head -1 || true)
  if [[ -z "${csv:-}" || ! -f "$csv" ]]; then
    echo "ERROR: no timing csv for $label" >&2
    return 1
  fi
  cp -a "$csv" "$OUT/time_consuming_${label}.csv"
  echo "saved $OUT/time_consuming_${label}.csv from $(basename "$csv")"
}

restore_cfg() {
  cp -a "$OUT/click_real_highspeed.yaml.bak" "$CFG"
  echo "restored yaml"
}

trap "stop_stack; restore_cfg" EXIT

echo "OUT=$OUT repeats=$REPEATS fly=${FLY_SEC}s goal=($GOAL_X,$GOAL_Y,$GOAL_Z)"

configure_route false
for i in $(seq 1 "$REPEATS"); do
  run_one "dense_r${i}"
done

configure_route true
for i in $(seq 1 "$REPEATS"); do
  run_one "hull_r${i}"
done

python3 - "$OUT" "$REPEATS" <<'PY'
import csv, math, sys
from pathlib import Path

out = Path(sys.argv[1])
repeats = int(sys.argv[2])
modes = [("dense", [f"dense_r{i}" for i in range(1, repeats + 1)]),
         ("hull", [f"hull_r{i}" for i in range(1, repeats + 1)])]

def vals(rows, k):
    v = []
    for r in rows:
        s = (r.get(k) or "").strip()
        if not s:
            continue
        try:
            v.append(float(s))
        except ValueError:
            pass
    return v

def summarize_file(path):
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return None
    def mean(k):
        v = vals(rows, k)
        return sum(v) / len(v) if v else float("nan")
    def rate(k):
        v = vals(rows, k)
        return sum(1 for x in v if x > 0.5) / len(v) if v else float("nan")
    mode = (rows[-1].get("EXP_COST_MODE") or "").strip()
    return {
        "n": len(rows),
        "mode": mode,
        "opt_ms_mean": 1000.0 * mean("EXP_TRAJ_OPT"),
        "opt_ms_sum": 1000.0 * sum(vals(rows, "EXP_TRAJ_OPT")),
        "replan_ms_mean": 1000.0 * mean("TOTAL_REPLAN"),
        "lbfgs_ms_mean": mean("EXP_LBFGS_MS"),
        "evals_mean": mean("EXP_EVALUATIONS"),
        "iters_mean": mean("EXP_LBFGS_ITERATIONS"),
        "fast_stop": rate("EXP_FAST_STOP"),
        "cont": rate("EXP_CONTINUOUS_FEASIBLE"),
        "viol": mean("EXP_MAX_NORMALIZED_VIOLATION"),
        "margin": mean("EXP_MIN_POSITION_MARGIN"),
        "dense_ms": mean("EXP_DENSE_INTEGRAL_MS"),
        "ctrl_ms": mean("EXP_CONTROL_POINT_FUNCTIONAL_MS"),
        "hull_checks": mean("EXP_HULL_CONTROL_CHECKS_PER_EVAL"),
    }

def nanmean(xs):
    xs = [x for x in xs if isinstance(x, float) and not math.isnan(x)]
    return sum(xs) / len(xs) if xs else float("nan")

print("\n=== per-run summary ===")
print(f"{'label':12s} {'mode':40s} {'calls':>5s} {'opt/c':>8s} {'lbfgs/c':>8s} {'evals':>7s} {'iters':>7s} {'fast%':>7s} {'cont%':>7s} {'viol':>8s}")
all_stats = {}
for kind, labels in modes:
    stats = []
    for label in labels:
        path = out / f"time_consuming_{label}.csv"
        s = summarize_file(path)
        if s is None:
            print(f"{label:12s} MISSING")
            continue
        stats.append(s)
        pct = lambda x: "n/a" if isinstance(x, float) and math.isnan(x) else f"{100*x:5.1f}%"
        print(f"{label:12s} {s['mode'][:40]:40s} {s['n']:5d} {s['opt_ms_mean']:8.2f} {s['lbfgs_ms_mean']:8.2f} {s['evals_mean']:7.1f} {s['iters_mean']:7.1f} {pct(s['fast_stop']):>7s} {pct(s['cont']):>7s} {s['viol']:8.3f}")
    all_stats[kind] = stats

print("\n=== average over 5 repeats (mean of per-run means) ===")
print(f"{'route':8s} {'opt_ms/call':>12s} {'lbfgs_ms':>10s} {'replan_ms':>10s} {'evals':>8s} {'iters':>8s} {'fast%':>8s} {'cont%':>8s} {'viol':>8s} {'calls':>7s}")
agg = {}
for kind, _ in modes:
    ss = all_stats.get(kind, [])
    if not ss:
        continue
    a = {
        "opt": nanmean([s["opt_ms_mean"] for s in ss]),
        "lbfgs": nanmean([s["lbfgs_ms_mean"] for s in ss]),
        "replan": nanmean([s["replan_ms_mean"] for s in ss]),
        "evals": nanmean([s["evals_mean"] for s in ss]),
        "iters": nanmean([s["iters_mean"] for s in ss]),
        "fast": nanmean([s["fast_stop"] for s in ss]),
        "cont": nanmean([s["cont"] for s in ss]),
        "viol": nanmean([s["viol"] for s in ss]),
        "calls": nanmean([float(s["n"]) for s in ss]),
        "dense": nanmean([s["dense_ms"] for s in ss]),
        "ctrl": nanmean([s["ctrl_ms"] for s in ss]),
    }
    agg[kind] = a
    print(f"{kind:8s} {a['opt']:12.2f} {a['lbfgs']:10.2f} {a['replan']:10.2f} {a['evals']:8.1f} {a['iters']:8.1f} {100*a['fast']:7.1f}% {100*a['cont']:7.1f}% {a['viol']:8.3f} {a['calls']:7.1f}")

if "dense" in agg and "hull" in agg:
    d, h = agg["dense"], agg["hull"]
    def ratio(a, b):
        return "n/a" if b == 0 or math.isnan(b) or math.isnan(a) else f"{a/b:.3f}x ({(a-b)/b*100:+.1f}%)"
    def speedup(a, b):
        # a=hull, b=dense; speedup = dense/hull
        return "n/a" if a == 0 or math.isnan(a) or math.isnan(b) else f"{b/a:.2f}x"
    print("\nhull vs dense:")
    print(f"  opt_ms/call : {ratio(h['opt'], d['opt'])}, speedup={speedup(h['opt'], d['opt'])}")
    print(f"  lbfgs_ms    : {ratio(h['lbfgs'], d['lbfgs'])}, speedup={speedup(h['lbfgs'], d['lbfgs'])}")
    print(f"  replan_ms   : {ratio(h['replan'], d['replan'])}, speedup={speedup(h['replan'], d['replan'])}")
    print(f"  evals/call  : {ratio(h['evals'], d['evals'])}")
    print(f"  iters/call  : {ratio(h['iters'], d['iters'])}")
    print(f"  dense_ms    : dense={d['dense']:.2f}, hull={h['dense']:.2f}")
    print(f"  ctrl_ms     : dense={d['ctrl']:.2f}, hull={h['ctrl']:.2f}")

# also write a small csv aggregate
agg_path = out / "summary_repeat5.csv"
with agg_path.open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["route", "repeat", "calls", "mode", "opt_ms_mean", "lbfgs_ms_mean",
                "replan_ms_mean", "evals_mean", "iters_mean", "fast_stop",
                "cont_feas", "viol_mean"])
    for kind, labels in modes:
        for i, label in enumerate(labels, 1):
            path = out / f"time_consuming_{label}.csv"
            s = summarize_file(path)
            if not s:
                continue
            w.writerow([kind, i, s["n"], s["mode"], f"{s['opt_ms_mean']:.6f}",
                        f"{s['lbfgs_ms_mean']:.6f}", f"{s['replan_ms_mean']:.6f}",
                        f"{s['evals_mean']:.6f}", f"{s['iters_mean']:.6f}",
                        f"{s['fast_stop']:.6f}", f"{s['cont']:.6f}", f"{s['viol']:.6f}"])
print(f"\nArtifacts: {out}")
print(f"Summary CSV: {agg_path}")
PY
