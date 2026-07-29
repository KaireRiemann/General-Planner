#!/usr/bin/env bash
# Four-route same-goal A/B, each run until FSM returns to WAIT_GOAL.
# Routes:
#   1) classic_dense      : LBFGS classic + dense, no warm start
#   2) fast_dense         : Fast LBFGS + dense, no warm start
#   3) fast_ws_dense      : Fast LBFGS + warm start + dense
#   4) full_hull          : Fast LBFGS + warm start + depth-2 Bezier hull
set -eo pipefail

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/devel/setup.bash
set -u

GOAL_X=${GOAL_X:-69.032}
GOAL_Y=${GOAL_Y:-1.901}
GOAL_Z=${GOAL_Z:-1.500}
MAX_WAIT_SEC=${MAX_WAIT_SEC:-180}
GOAL_DIST_THRESH=${GOAL_DIST_THRESH:-0.35}

CFG=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/config/click_real_highspeed.yaml
LOGDIR=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log
OUT=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log/four_route_wait_goal_$(date +%Y%m%d_%H%M%S)
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
  local label="$1" hull="$2" fast="$3" warm="$4"
  echo "===== configure $label hull=$hull fast=$fast warm=$warm ====="
  # Main planner YAML only toggles representation/solver switches. Hull
  # formulation lives in traj_opt/convex_hull/click_real_highspeed.yaml.
  set_yaml_key convex_hull_en "$hull"
  set_yaml_key lbfgs_fast_en "$fast"
  set_yaml_key lbfgs_fast_phase0_guards_en false
  set_yaml_key lbfgs_warm_start_en "$warm"
  grep -E "convex_hull_en:|lbfgs_fast_en:|lbfgs_warm_start_en:|convex_hull_config:" "$CFG"
}

# Wait until FSM has entered FOLLOW_TRAJ, then returned to WAIT_GOAL.
# Fallback: odom within GOAL_DIST_THRESH of goal for 2 consecutive seconds.
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
        # After flying, WAIT_GOAL means mission finished / waiting again.
        if python3 - "$launch_log" <<'PY'
import sys, re
from pathlib import Path
text = Path(sys.argv[1]).read_text(errors="ignore")
# Find last FOLLOW then a later WAIT_GOAL print.
states = [(m.start(), m.group(1)) for m in re.finditer(
    r"Current state:\s*(FOLLOW_TRAJ|WAIT_GOAL|GENERATE_TRAJ)", text)]
saw_f = False
for _, s in states:
    if s == "FOLLOW_TRAJ":
        saw_f = True
    elif s == "WAIT_GOAL" and saw_f:
        # require that FOLLOW appeared before this WAIT
        sys.exit(0)
sys.exit(1)
PY
        then
          echo "detected WAIT_GOAL after FOLLOW_TRAJ (elapsed $((now - t0))s)"
          return 0
        fi
      fi
    fi

    # Odometry fallback
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

  # settle so last csv rows flush
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

echo "OUT=$OUT goal=($GOAL_X,$GOAL_Y,$GOAL_Z) max_wait=${MAX_WAIT_SEC}s"

configure_route classic_dense false false false
run_one classic_dense || true

configure_route fast_dense false true false
run_one fast_dense || true

configure_route fast_ws_dense false true true
run_one fast_ws_dense || true

configure_route full_hull true true true
run_one full_hull || true

python3 - "$OUT" <<'PY'
import csv, math, sys
from pathlib import Path

out = Path(sys.argv[1])
labels = ["classic_dense", "fast_dense", "fast_ws_dense", "full_hull"]

def vals(rows, k):
    v=[]
    for r in rows:
        s=(r.get(k) or "").strip()
        if not s: continue
        try: v.append(float(s))
        except ValueError: pass
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
        "replan_ms_mean": 1000*mean("TOTAL_REPLAN"),
        "replan_ms_sum": 1000*summ("TOTAL_REPLAN"),
        "evals_mean": mean("EXP_EVALUATIONS"),
        "iters_mean": mean("EXP_LBFGS_ITERATIONS"),
        "fast": rate("EXP_FAST_STOP"),
        "cont": rate("EXP_CONTINUOUS_FEASIBLE"),
        "viol": mean("EXP_MAX_NORMALIZED_VIOLATION"),
        "ws_attempt": rate("EXP_WARM_START_ATTEMPTED"),
        "ws_accept": rate("EXP_WARM_START_ACCEPTED"),
        "dense_ms": mean("EXP_DENSE_INTEGRAL_MS"),
        "ctrl_ms": mean("EXP_CONTROL_POINT_FUNCTIONAL_MS"),
    }

print("\n=== four-route wait-goal comparison ===")
print(f"{'label':14s} {'mode':42s} {'calls':>5s} {'wall_s':>7s} {'opt/c':>7s} {'opt_sum':>8s} {'iters':>7s} {'fast%':>7s} {'ws_acc%':>8s} {'cont%':>7s} {'viol':>7s}")
stats=[]
for label in labels:
    s=summarize(label)
    if not s:
        print(f"{label:14s} MISSING")
        continue
    stats.append(s)
    pct=lambda x: "n/a" if isinstance(x,float) and math.isnan(x) else f"{100*x:5.1f}%"
    print(f"{s['label']:14s} {s['mode'][:42]:42s} {s['n']:5d} {s['wall']:7.1f} {s['opt_ms_mean']:7.2f} {s['opt_ms_sum']:8.1f} {s['iters_mean']:7.1f} {pct(s['fast']):>7s} {pct(s['ws_accept']):>8s} {pct(s['cont']):>7s} {s['viol']:7.3f}")

by={s['label']:s for s in stats}
base=by.get("classic_dense")
print("\nvs classic_dense:")
def r(a,b):
    if a is None or b is None or b==0 or math.isnan(a) or math.isnan(b): return "n/a"
    return f"{a/b:.3f}x ({(a-b)/b*100:+.1f}%)"
def sp(a,b):
    if a is None or b is None or a==0 or math.isnan(a) or math.isnan(b): return "n/a"
    return f"{b/a:.2f}x"
if base:
    for label in labels[1:]:
        s=by.get(label)
        if not s: continue
        print(f"  {label}:")
        print(f"    opt/c   {r(s['opt_ms_mean'], base['opt_ms_mean'])}, speedup={sp(s['opt_ms_mean'], base['opt_ms_mean'])}")
        print(f"    opt_sum {r(s['opt_ms_sum'], base['opt_ms_sum'])}, speedup={sp(s['opt_ms_sum'], base['opt_ms_sum'])}")
        print(f"    iters   {r(s['iters_mean'], base['iters_mean'])}")
        print(f"    wall_s  {r(s['wall'], base['wall'])}")
        print(f"    replan_sum {r(s['replan_ms_sum'], base['replan_ms_sum'])}")

# pairwise full_hull vs fast_ws_dense
if "full_hull" in by and "fast_ws_dense" in by:
    h,d=by["full_hull"], by["fast_ws_dense"]
    print("\nfull_hull vs fast_ws_dense:")
    print(f"  opt/c   {r(h['opt_ms_mean'], d['opt_ms_mean'])}, speedup={sp(h['opt_ms_mean'], d['opt_ms_mean'])}")
    print(f"  opt_sum {r(h['opt_ms_sum'], d['opt_ms_sum'])}")
    print(f"  cont%   hull={100*h['cont']:.1f}% dense_ws={100*d['cont']:.1f}%")

sum_path=out/"summary_four_route.csv"
with sum_path.open("w", newline="") as f:
    w=csv.writer(f)
    w.writerow(["label","mode","calls","wall_sec","opt_ms_mean","opt_ms_sum","lbfgs_ms_mean",
                "replan_ms_mean","replan_ms_sum","evals_mean","iters_mean","fast_stop",
                "cont_feas","viol_mean","ws_attempt","ws_accept","dense_ms","ctrl_ms"])
    for s in stats:
        w.writerow([s["label"], s["mode"], s["n"], f"{s['wall']:.6f}", f"{s['opt_ms_mean']:.6f}",
                    f"{s['opt_ms_sum']:.6f}", f"{s['lbfgs_ms_mean']:.6f}", f"{s['replan_ms_mean']:.6f}",
                    f"{s['replan_ms_sum']:.6f}", f"{s['evals_mean']:.6f}", f"{s['iters_mean']:.6f}",
                    f"{s['fast']:.6f}", f"{s['cont']:.6f}", f"{s['viol']:.6f}",
                    f"{s['ws_attempt']:.6f}", f"{s['ws_accept']:.6f}", f"{s['dense_ms']:.6f}",
                    f"{s['ctrl_ms']:.6f}"])
print(f"\nArtifacts: {out}")
print(f"Summary: {sum_path}")
PY
