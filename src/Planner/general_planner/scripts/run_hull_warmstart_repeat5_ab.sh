#!/usr/bin/env bash
# 5x same-goal hull runs with current warm-start config; compare to previous
# dense/hull baseline from dense_vs_hull_r5_20260727_063111.
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
BASELINE=${BASELINE:-/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log/dense_vs_hull_r5_20260727_063111}

CFG=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/config/click_real_highspeed.yaml
LOGDIR=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log
OUT=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log/hull_warmstart_r5_$(date +%Y%m%d_%H%M%S)
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

configure_hull() {
  set_yaml_key convex_hull_en true
  set_yaml_key convex_hull_basis 0
  set_yaml_key convex_hull_flatness_en true
  set_yaml_key convex_hull_require_certification false
  set_yaml_key lbfgs_fast_en true
  set_yaml_key lbfgs_fast_phase0_guards_en false
  set_yaml_key lbfgs_warm_start_en true
  grep -E "convex_hull_en:|lbfgs_fast_en:|lbfgs_warm_start_en:|convex_hull_flatness_en:" "$CFG"
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

echo "OUT=$OUT repeats=$REPEATS fly=${FLY_SEC}s baseline=$BASELINE"
configure_hull
for i in $(seq 1 "$REPEATS"); do
  run_one "hull_ws_r${i}"
done

python3 - "$OUT" "$REPEATS" "$BASELINE" <<'PY'
import csv, math, sys
from pathlib import Path

out = Path(sys.argv[1])
repeats = int(sys.argv[2])
baseline = Path(sys.argv[3])

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
    if not path.exists():
        return None
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return None
    def mean(k):
        v = vals(rows, k)
        return sum(v) / len(v) if v else float("nan")
    def rate(k):
        v = vals(rows, k)
        return sum(1 for x in v if x > 0.5) / len(v) if v else float("nan")
    def summ(k):
        return sum(vals(rows, k))
    mode = (rows[-1].get("EXP_COST_MODE") or "").strip()
    return {
        "n": len(rows),
        "mode": mode,
        "opt_ms_mean": 1000.0 * mean("EXP_TRAJ_OPT"),
        "opt_ms_sum": 1000.0 * summ("EXP_TRAJ_OPT"),
        "replan_ms_mean": 1000.0 * mean("TOTAL_REPLAN"),
        "replan_ms_sum": 1000.0 * summ("TOTAL_REPLAN"),
        "lbfgs_ms_mean": mean("EXP_LBFGS_MS"),
        "evals_mean": mean("EXP_EVALUATIONS"),
        "iters_mean": mean("EXP_LBFGS_ITERATIONS"),
        "fast_stop": rate("EXP_FAST_STOP"),
        "cont": rate("EXP_CONTINUOUS_FEASIBLE"),
        "viol": mean("EXP_MAX_NORMALIZED_VIOLATION"),
        "dense_ms": mean("EXP_DENSE_INTEGRAL_MS"),
        "ctrl_ms": mean("EXP_CONTROL_POINT_FUNCTIONAL_MS"),
        "ws_attempt": rate("EXP_WARM_START_ATTEMPTED"),
        "ws_accept": rate("EXP_WARM_START_ACCEPTED"),
        "ws_ms": mean("EXP_WARM_START_MS"),
    }

def nanmean(xs):
    xs = [x for x in xs if isinstance(x, float) and not math.isnan(x)]
    return sum(xs) / len(xs) if xs else float("nan")

def aggregate(label_paths):
    stats = []
    for p in label_paths:
        s = summarize_file(p)
        if s:
            stats.append(s)
    if not stats:
        return None
    return {
        "runs": len(stats),
        "calls": nanmean([float(s["n"]) for s in stats]),
        "opt": nanmean([s["opt_ms_mean"] for s in stats]),
        "opt_sum": nanmean([s["opt_ms_sum"] for s in stats]),
        "lbfgs": nanmean([s["lbfgs_ms_mean"] for s in stats]),
        "replan": nanmean([s["replan_ms_mean"] for s in stats]),
        "replan_sum": nanmean([s["replan_ms_sum"] for s in stats]),
        "evals": nanmean([s["evals_mean"] for s in stats]),
        "iters": nanmean([s["iters_mean"] for s in stats]),
        "fast": nanmean([s["fast_stop"] for s in stats]),
        "cont": nanmean([s["cont"] for s in stats]),
        "viol": nanmean([s["viol"] for s in stats]),
        "dense": nanmean([s["dense_ms"] for s in stats]),
        "ctrl": nanmean([s["ctrl_ms"] for s in stats]),
        "ws_attempt": nanmean([s["ws_attempt"] for s in stats]),
        "ws_accept": nanmean([s["ws_accept"] for s in stats]),
        "ws_ms": nanmean([s["ws_ms"] for s in stats]),
        "mode": stats[-1]["mode"],
        "per": stats,
    }

print("\n=== warm-start hull per-run ===")
print(f"{'label':12s} {'calls':>5s} {'opt/c':>8s} {'opt_sum':>8s} {'iters':>7s} {'fast%':>7s} {'cont%':>7s} {'ws_acc%':>8s} {'viol':>8s}")
ws_paths = []
for i in range(1, repeats + 1):
    p = out / f"time_consuming_hull_ws_r{i}.csv"
    ws_paths.append(p)
    s = summarize_file(p)
    if not s:
        print(f"hull_ws_r{i} MISSING")
        continue
    pct = lambda x: "n/a" if isinstance(x, float) and math.isnan(x) else f"{100*x:5.1f}%"
    print(f"hull_ws_r{i:<3d} {s['n']:5d} {s['opt_ms_mean']:8.2f} {s['opt_ms_sum']:8.1f} {s['iters_mean']:7.1f} {pct(s['fast_stop']):>7s} {pct(s['cont']):>7s} {pct(s['ws_accept']):>8s} {s['viol']:8.3f}")

now = aggregate(ws_paths)
prev_dense = aggregate([baseline / f"time_consuming_dense_r{i}.csv" for i in range(1, repeats + 1)])
prev_hull = aggregate([baseline / f"time_consuming_hull_r{i}.csv" for i in range(1, repeats + 1)])

rows = [
    ("dense@prev", prev_dense),
    ("hull@prev", prev_hull),
    ("hull+ws@now", now),
]

print("\n=== average over 5 repeats ===")
print(f"{'route':12s} {'opt/c':>8s} {'opt_sum':>8s} {'lbfgs':>7s} {'replan':>7s} {'evals':>7s} {'iters':>7s} {'fast%':>7s} {'cont%':>7s} {'viol':>7s} {'calls':>6s} {'ws_acc%':>8s}")
for name, a in rows:
    if not a:
        print(f"{name:12s} MISSING")
        continue
    print(f"{name:12s} {a['opt']:8.2f} {a['opt_sum']:8.1f} {a['lbfgs']:7.2f} {a['replan']:7.2f} {a['evals']:7.1f} {a['iters']:7.1f} {100*a['fast']:6.1f}% {100*a['cont']:6.1f}% {a['viol']:7.3f} {a['calls']:6.1f} {100*a['ws_accept']:7.1f}%")

def ratio(a, b):
    if a is None or b is None or b == 0 or math.isnan(a) or math.isnan(b):
        return "n/a"
    return f"{a/b:.3f}x ({(a-b)/b*100:+.1f}%)"

def speedup(a, b):
    # a=new, b=old; speedup = old/new
    if a is None or b is None or a == 0 or math.isnan(a) or math.isnan(b):
        return "n/a"
    return f"{b/a:.2f}x"

if now and prev_dense and prev_hull:
    print("\nhull+warmstart vs previous:")
    print(f"  vs dense@prev  opt/c={ratio(now['opt'], prev_dense['opt'])}, speedup={speedup(now['opt'], prev_dense['opt'])}, opt_sum={ratio(now['opt_sum'], prev_dense['opt_sum'])}, iters={ratio(now['iters'], prev_dense['iters'])}")
    print(f"  vs hull@prev   opt/c={ratio(now['opt'], prev_hull['opt'])}, speedup={speedup(now['opt'], prev_hull['opt'])}, opt_sum={ratio(now['opt_sum'], prev_hull['opt_sum'])}, iters={ratio(now['iters'], prev_hull['iters'])}")
    print(f"  warm-start accept rate={100*now['ws_accept']:.1f}%  attempt={100*now['ws_attempt']:.1f}%  overhead={now['ws_ms']:.3f} ms/call")
    print(f"  cont_feas: dense={100*prev_dense['cont']:.1f}%  hull={100*prev_hull['cont']:.1f}%  hull+ws={100*now['cont']:.1f}%")
    print(f"  mode now: {now['mode']}")

# write summary csv
agg_path = out / "summary_vs_baseline.csv"
with agg_path.open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["route", "opt_ms_mean", "opt_ms_sum", "lbfgs_ms_mean", "replan_ms_mean",
                "evals_mean", "iters_mean", "fast_stop", "cont_feas", "viol_mean",
                "calls_mean", "ws_accept", "ws_attempt", "ws_ms"])
    for name, a in rows:
        if not a:
            continue
        w.writerow([name, f"{a['opt']:.6f}", f"{a['opt_sum']:.6f}", f"{a['lbfgs']:.6f}",
                    f"{a['replan']:.6f}", f"{a['evals']:.6f}", f"{a['iters']:.6f}",
                    f"{a['fast']:.6f}", f"{a['cont']:.6f}", f"{a['viol']:.6f}",
                    f"{a['calls']:.6f}", f"{a['ws_accept']:.6f}", f"{a['ws_attempt']:.6f}",
                    f"{a['ws_ms']:.6f}"])
print(f"\nArtifacts: {out}")
print(f"Summary: {agg_path}")
PY
