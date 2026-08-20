#!/usr/bin/env bash
# Real State2State A/B: Euclidean (minco_metric_mode=0) vs a metric chart.
# Default: Frozen MCE V1 (mode=1). For full space-time joint whitening use
#   MODE_B=4 LABEL_B=frozen_joint
# The forest, goal, corridor, hull, Fast L-BFGS and objective weights stay
# identical. Only minco_metric_mode changes. Default yaml stays mode 0.
#
# Usage (on a ROS1 Noetic workspace that can launch click_demo):
#   bash scripts/run_mce_vs_euclidean_state2state_ab.sh
#   MODE_B=4 LABEL_B=frozen_joint bash scripts/run_mce_vs_euclidean_state2state_ab.sh
#
# Optional env:
#   GOAL_X GOAL_Y GOAL_Z MAX_WAIT_SEC GOAL_DIST_THRESH WS_ROOT
#   MODE_A MODE_B LABEL_A LABEL_B
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_DIR=$(cd "$SCRIPT_DIR/.." && pwd)

find_workspace() {
  local d="$PKG_DIR"
  while [[ "$d" != "/" ]]; do
    if [[ -f "$d/devel/setup.bash" ]]; then
      printf '%s\n' "$d"
      return 0
    fi
    d=$(dirname "$d")
  done
  return 1
}

WS_ROOT=${WS_ROOT:-$(find_workspace || true)}
if [[ -z "${WS_ROOT:-}" ]]; then
  echo "ERROR: cannot find a catkin devel/setup.bash above $PKG_DIR" >&2
  echo "Set WS_ROOT to the ROS1 workspace root." >&2
  exit 1
fi

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
else
  echo "ERROR: /opt/ros/noetic/setup.bash is missing." >&2
  echo "This A/B needs ROS1 Noetic plus a rebuilt fsm_node that includes Frozen MCE V1." >&2
  echo "This host only has: $(ls /opt/ros 2>/dev/null | tr '\n' ' ')" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$WS_ROOT/devel/setup.bash"
set -u

if ! command -v roslaunch >/dev/null || ! command -v fsm_node >/dev/null; then
  echo "ERROR: roslaunch/fsm_node not on PATH after sourcing $WS_ROOT/devel/setup.bash" >&2
  exit 1
fi
if ! ldd "$(command -v fsm_node)" | grep -q 'libroscpp.so => /'; then
  echo "ERROR: fsm_node is not a runnable ROS1 binary on this host (missing libroscpp)." >&2
  exit 1
fi

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}

GOAL_X=${GOAL_X:-69.032}
GOAL_Y=${GOAL_Y:-1.901}
GOAL_Z=${GOAL_Z:-1.500}
MAX_WAIT_SEC=${MAX_WAIT_SEC:-180}
GOAL_DIST_THRESH=${GOAL_DIST_THRESH:-0.35}
MODE_A=${MODE_A:-0}
MODE_B=${MODE_B:-1}
LABEL_A=${LABEL_A:-euclidean}
LABEL_B=${LABEL_B:-frozen_mce}

CFG="$PKG_DIR/config/click_real_highspeed.yaml"
LOGDIR="$PKG_DIR/log"
OUT="$LOGDIR/mce_vs_euclidean_state2state_$(date +%Y%m%d_%H%M%S)"
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
  local label="$1" mode="$2"
  echo "===== configure $label minco_metric_mode=$mode ====="
  set_yaml_key minco_metric_mode "$mode"
  grep -E "minco_metric_mode:|convex_hull_en:|lbfgs_fast_en:|lbfgs_warm_start_en:" "$CFG"
}

wait_until_wait_goal() {
  local launch_log="$1"
  local t0
  t0=$(date +%s)
  local saw_follow=0
  local near_streak=0

  while true; do
    local now
    now=$(date +%s)
    if (( now - t0 > MAX_WAIT_SEC )); then
      echo "TIMEOUT after ${MAX_WAIT_SEC}s waiting for WAIT_GOAL" >&2
      return 1
    fi

    if [[ -f "$launch_log" ]]; then
      if grep -q "Current state: FOLLOW_TRAJ" "$launch_log"; then
        saw_follow=1
      fi
      if (( saw_follow == 1 )); then
        if python3 - "$launch_log" <<'PY'
import sys, re
from pathlib import Path
text = Path(sys.argv[1]).read_text(errors="ignore")
states = [(m.start(), m.group(1)) for m in re.finditer(
    r"Current state:\s*(FOLLOW_TRAJ|WAIT_GOAL|GENERATE_TRAJ)", text)]
saw_f = False
for _, s in states:
    if s == "FOLLOW_TRAJ":
        saw_f = True
    elif s == "WAIT_GOAL" and saw_f:
        sys.exit(0)
sys.exit(1)
PY
        then
          echo "detected WAIT_GOAL after FOLLOW_TRAJ (elapsed $((now - t0))s)"
          return 0
        fi
      fi
    fi

    if python3 - "$GOAL_X" "$GOAL_Y" "$GOAL_Z" "$GOAL_DIST_THRESH" <<'PY'
import sys, math
try:
    import rospy
    from nav_msgs.msg import Odometry
except Exception:
    sys.exit(2)
gx, gy, gz, thr = map(float, sys.argv[1:5])
if not rospy.core.is_initialized():
    rospy.init_node("wait_goal_probe", anonymous=True, disable_signals=True)
try:
    msg = rospy.wait_for_message("/lidar_slam/odom", Odometry, timeout=1.0)
except Exception:
    sys.exit(2)
p = msg.pose.pose.position
d = math.sqrt((p.x-gx)**2 + (p.y-gy)**2 + (p.z-gz)**2)
sys.exit(0 if d <= thr else 1)
PY
    then
      near_streak=$((near_streak + 1))
      if (( near_streak >= 2 )) && (( saw_follow == 1 || now - t0 > 15 )); then
        echo "detected near-goal odom fallback (elapsed $((now - t0))s)"
        return 0
      fi
    else
      near_streak=0
    fi
    sleep 1
  done
}

run_one() {
  local label="$1"
  echo "===== run $label ====="
  find "$LOGDIR" -maxdepth 1 -name 'time_consuming_*.csv' -delete 2>/dev/null || true

  local launch_log="$OUT/${label}_launch.log"
  roscore >"$OUT/${label}_roscore.log" 2>&1 &
  echo $! >"$OUT/roscore.pid"
  sleep 2
  roslaunch task_planner click_demo.launch rviz:=false fpv_rviz:=false \
    >"$launch_log" 2>&1 &
  echo $! >"$OUT/launch.pid"

  for i in $(seq 1 60); do
    if rostopic list 2>/dev/null | grep -q "/lidar_slam/odom"; then
      break
    fi
    sleep 1
  done
  sleep 3

  local wall_begin
  wall_begin=$(date +%s.%N)
  echo "publish goal ($GOAL_X, $GOAL_Y, $GOAL_Z)"
  timeout 2 rostopic pub /goal geometry_msgs/PoseStamped \
    "{header: {frame_id: \"world\"}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {w: 1.0}}}" \
    -r 20 >/dev/null 2>&1 || true

  local status=0
  wait_until_wait_goal "$launch_log" || status=$?
  local wall_end
  wall_end=$(date +%s.%N)
  python3 - <<PY
begin=float("$wall_begin"); end=float("$wall_end")
print(f"wall_mission_sec={end-begin:.3f} status={$status}")
open("$OUT/${label}_wall_sec.txt","w").write(f"{end-begin:.6f}\n")
PY

  sleep 2
  stop_stack

  local csv
  csv=$(ls -t "$LOGDIR"/time_consuming_*.csv 2>/dev/null | head -1 || true)
  if [[ -z "${csv:-}" || ! -f "$csv" ]]; then
    echo "ERROR: no timing csv for $label" >&2
    return 1
  fi
  cp -a "$csv" "$OUT/time_consuming_${label}.csv"
  echo "saved $OUT/time_consuming_${label}.csv from $(basename "$csv")"
  return "$status"
}

restore_cfg() {
  cp -a "$OUT/click_real_highspeed.yaml.bak" "$CFG"
  echo "restored yaml"
}

trap "stop_stack; restore_cfg" EXIT

echo "OUT=$OUT WS_ROOT=$WS_ROOT goal=($GOAL_X,$GOAL_Y,$GOAL_Z) max_wait=${MAX_WAIT_SEC}s"
echo "Only minco_metric_mode is toggled ($LABEL_A=$MODE_A vs $LABEL_B=$MODE_B). Hull / Fast L-BFGS / warm-start stay as in yaml."

configure_route "$LABEL_A" "$MODE_A"
run_one "$LABEL_A" || true

configure_route "$LABEL_B" "$MODE_B"
run_one "$LABEL_B" || true

python3 - "$OUT" "$LABEL_A" "$LABEL_B" <<'PY'
import csv, math, sys
from pathlib import Path

out = Path(sys.argv[1])
labels = [sys.argv[2], sys.argv[3]]

def vals(rows, k):
    v=[]
    for r in rows:
        s=(r.get(k) or "").strip()
        if not s:
            continue
        try:
            v.append(float(s))
        except ValueError:
            pass
    return v

def summarize(label):
    path = out / f"time_consuming_{label}.csv"
    wall_path = out / f"{label}_wall_sec.txt"
    wall = float(wall_path.read_text()) if wall_path.exists() else float("nan")
    if not path.exists():
        return None
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return None
    def mean(k):
        v=vals(rows,k); return sum(v)/len(v) if v else float("nan")
    def rate(k):
        v=vals(rows,k); return sum(1 for x in v if x>0.5)/len(v) if v else float("nan")
    def summ(k):
        return sum(vals(rows,k))
    mode=(rows[-1].get("EXP_COST_MODE") or "").strip()
    return {
        "label": label,
        "mode": mode,
        "n": len(rows),
        "wall": wall,
        "opt_ms_mean": 1000*mean("EXP_TRAJ_OPT"),
        "opt_ms_sum": 1000*summ("EXP_TRAJ_OPT"),
        "lbfgs_ms_mean": mean("EXP_LBFGS_MS"),
        "lbfgs_ms_sum": summ("EXP_LBFGS_MS"),
        "metric_ms_mean": mean("EXP_METRIC_MS"),
        "metric_ms_sum": summ("EXP_METRIC_MS"),
        "metric_hit": rate("EXP_METRIC_CACHE_HIT"),
        "metric_refresh": mean("EXP_METRIC_REFRESH") if any((r.get("EXP_METRIC_REFRESH") or "").strip() for r in rows) else float("nan"),
        "replan_ms_mean": 1000*mean("TOTAL_REPLAN"),
        "replan_ms_sum": 1000*summ("TOTAL_REPLAN"),
        "evals_mean": mean("EXP_EVALUATIONS"),
        "iters_mean": mean("EXP_LBFGS_ITERATIONS"),
        "fast": rate("EXP_FAST_STOP"),
        "cont": rate("EXP_CONTINUOUS_FEASIBLE"),
        "viol": mean("EXP_MAX_NORMALIZED_VIOLATION"),
        "stat": mean("EXP_STATIONARITY_RESIDUAL"),
        "dense_ms": mean("EXP_DENSE_INTEGRAL_MS"),
        "ctrl_ms": mean("EXP_CONTROL_POINT_FUNCTIONAL_MS"),
    }

print("\n=== State2State metric A/B ===")
print(f"{'label':12s} {'mode':28s} {'calls':>5s} {'wall_s':>7s} {'opt/c':>7s} {'opt_sum':>8s} {'lbfgs':>7s} {'metric':>7s} {'iters':>7s} {'evals':>7s} {'fast%':>7s} {'cont%':>7s} {'viol':>7s}")
stats=[]
for label in labels:
    s=summarize(label)
    if not s:
        print(f"{label:12s} MISSING")
        continue
    stats.append(s)
    pct=lambda x: "n/a" if isinstance(x,float) and math.isnan(x) else f"{100*x:5.1f}%"
    print(f"{s['label']:12s} {s['mode'][:28]:28s} {s['n']:5d} {s['wall']:7.1f} {s['opt_ms_mean']:7.2f} {s['opt_ms_sum']:8.1f} {s['lbfgs_ms_mean']:7.2f} {s['metric_ms_mean']:7.3f} {s['iters_mean']:7.1f} {s['evals_mean']:7.1f} {pct(s['fast']):>7s} {pct(s['cont']):>7s} {s['viol']:7.3f}")

by={s['label']:s for s in stats}
base=by.get(labels[0])
mce=by.get(labels[1])

def r(a,b):
    if a is None or b is None or b==0 or math.isnan(a) or math.isnan(b):
        return "n/a"
    return f"{a/b:.3f}x ({(a-b)/b*100:+.1f}%)"
def sp(a,b):
    if a is None or b is None or a==0 or math.isnan(a) or math.isnan(b):
        return "n/a"
    return f"{b/a:.2f}x"

if base and mce:
    print(f"\n{labels[1]} vs {labels[0]} (same forest/goal/corridor/objective):")
    print(f"  opt/call     {r(mce['opt_ms_mean'], base['opt_ms_mean'])}, speedup={sp(mce['opt_ms_mean'], base['opt_ms_mean'])}")
    print(f"  opt_sum      {r(mce['opt_ms_sum'], base['opt_ms_sum'])}, speedup={sp(mce['opt_ms_sum'], base['opt_ms_sum'])}")
    print(f"  lbfgs/call   {r(mce['lbfgs_ms_mean'], base['lbfgs_ms_mean'])}, speedup={sp(mce['lbfgs_ms_mean'], base['lbfgs_ms_mean'])}")
    print(f"  replan/call  {r(mce['replan_ms_mean'], base['replan_ms_mean'])}")
    print(f"  replan_sum   {r(mce['replan_ms_sum'], base['replan_ms_sum'])}")
    print(f"  iters        {r(mce['iters_mean'], base['iters_mean'])}")
    print(f"  evals        {r(mce['evals_mean'], base['evals_mean'])}")
    print(f"  wall_s       {r(mce['wall'], base['wall'])}")
    print(f"  metric/call  {labels[0]}={base['metric_ms_mean']:.3f} ms  {labels[1]}={mce['metric_ms_mean']:.3f} ms  cache={100*mce['metric_hit']:.1f}%  refresh={mce.get('metric_refresh', float('nan'))}")
    print(f"  quality      viol {labels[0]}={base['viol']:.4g} {labels[1]}={mce['viol']:.4g};  cont% {labels[0]}={100*base['cont']:.1f} {labels[1]}={100*mce['cont']:.1f}")

sum_path=out/"summary_mce_vs_euclidean.csv"
with sum_path.open("w", newline="") as f:
    w=csv.writer(f)
    w.writerow(["label","mode","calls","wall_sec","opt_ms_mean","opt_ms_sum","lbfgs_ms_mean","lbfgs_ms_sum",
                "metric_ms_mean","metric_ms_sum","metric_cache_hit","metric_refresh","replan_ms_mean","replan_ms_sum",
                "evals_mean","iters_mean","fast_stop","cont_feas","viol_mean","stationarity_mean",
                "dense_ms","ctrl_ms"])
    for s in stats:
        w.writerow([s["label"], s["mode"], s["n"], f"{s['wall']:.6f}", f"{s['opt_ms_mean']:.6f}",
                    f"{s['opt_ms_sum']:.6f}", f"{s['lbfgs_ms_mean']:.6f}", f"{s['lbfgs_ms_sum']:.6f}",
                    f"{s['metric_ms_mean']:.6f}", f"{s['metric_ms_sum']:.6f}", f"{s['metric_hit']:.6f}",
                    f"{s.get('metric_refresh', float('nan')):.6f}",
                    f"{s['replan_ms_mean']:.6f}", f"{s['replan_ms_sum']:.6f}", f"{s['evals_mean']:.6f}",
                    f"{s['iters_mean']:.6f}", f"{s['fast']:.6f}", f"{s['cont']:.6f}", f"{s['viol']:.6f}",
                    f"{s['stat']:.6f}", f"{s['dense_ms']:.6f}", f"{s['ctrl_ms']:.6f}"])
print(f"\nArtifacts: {out}")
print(f"Summary: {sum_path}")
PY
