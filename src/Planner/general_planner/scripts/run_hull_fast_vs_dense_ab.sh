#!/usr/bin/env bash
# Speed check: dense vs single-solve depth-2 Bezier hull (no Phase-2).
set -eo pipefail

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/devel/setup.bash
set -u

CFG=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/config/click_real_highspeed.yaml
LOGDIR=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log
OUT=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log/hull_fast_ab_$(date +%Y%m%d_%H%M%S)
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

run_one() {
  local label="$1" mode_expect="$2" hull="$3"
  echo "===== $label ====="
  set_yaml_key convex_hull_en "$hull"
  set_yaml_key convex_hull_alm_en false
  set_yaml_key convex_hull_phase2_corrector_en false
  set_yaml_key convex_hull_adaptive_en false
  set_yaml_key convex_hull_subdivision_depth 2
  set_yaml_key convex_hull_cost_version 2
  # Keep flatness hull OFF to match pre-flatness residual dense omg/thr path.
  set_yaml_key convex_hull_flatness_en false
  set_yaml_key convex_hull_flatness_major_iters 1
  set_yaml_key lbfgs_fast_en true
  set_yaml_key lbfgs_fast_phase0_guards_en false
  grep -E "convex_hull_en:|convex_hull_flatness_en:|convex_hull_subdivision_depth:|convex_hull_phase2_corrector_en:|lbfgs_fast_en:" "$CFG"

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

  echo "publish goal"
  timeout 2 rostopic pub /goal geometry_msgs/PoseStamped \
    "{header: {frame_id: \"world\"}, pose: {position: {x: 69.032, y: 1.901, z: 1.500}, orientation: {w: 1.0}}}" \
    -r 20 >/dev/null 2>&1 || true

  echo "fly 25s"
  sleep 25
  stop_stack

  local csv="$LOGDIR/time_consuming_${mode_expect}.csv"
  if [[ ! -f "$csv" ]]; then
    csv=$(ls -t "$LOGDIR"/time_consuming_*.csv 2>/dev/null | head -1 || true)
  fi
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

run_one fast_dense dense false
run_one hull_d2 convex_bezier_v2_d2 true

python3 - "$OUT" <<'PY'
import csv, math, sys
from pathlib import Path
out = Path(sys.argv[1])
labels = ["fast_dense", "hull_d2"]

def summarize(path):
    rows = list(csv.DictReader(path.open()))
    def vals(k):
        v=[]
        for r in rows:
            s=(r.get(k) or "").strip()
            if not s: continue
            try: v.append(float(s))
            except ValueError: pass
        return v
    def fsum(k): return sum(vals(k))
    def fmean(k):
        v=vals(k); return sum(v)/len(v) if v else float("nan")
    def frate(k):
        v=vals(k); return (sum(1 for x in v if x>0.5)/len(v)) if v else float("nan")
    return {
        "rows": len(rows),
        "mode": (rows[-1].get("EXP_COST_MODE") or "").strip() if rows else "",
        "opt_ms": 1000*fsum("EXP_TRAJ_OPT"),
        "lbfgs_ms": fsum("EXP_LBFGS_MS"),
        "ctrl_ms": fsum("EXP_CONTROL_POINT_FUNCTIONAL_MS"),
        "dense_ms": fsum("EXP_DENSE_INTEGRAL_MS"),
        "evals": fsum("EXP_EVALUATIONS"),
        "e_call": fmean("EXP_EVALUATIONS"),
        "hull": fmean("EXP_HULL_CONTROL_CHECKS_PER_EVAL"),
        "p2": frate("EXP_PHASE2_TRIGGERED"),
        "cont": frate("EXP_CONTINUOUS_FEASIBLE"),
        "nviol": fmean("EXP_MAX_NORMALIZED_VIOLATION"),
        "margin": fmean("EXP_MIN_POSITION_MARGIN"),
        "opt_per": (1000*fsum("EXP_TRAJ_OPT")/len(rows)) if rows else float("nan"),
        "fast": frate("EXP_FAST_STOP"),
    }

print("\n=== hull depth-2 (dense omg/thr residual) vs dense ===")
print(f"{'label':12s} {'mode':24s} {'calls':>5s} {'opt_ms':>9s} {'opt/call':>8s} {'ctrl':>8s} {'dense':>8s} {'cont%':>7s} {'viol':>8s} {'margin':>8s}")
results=[]
for label in labels:
    s=summarize(out/f"time_consuming_{label}.csv")
    results.append((label,s))
    pct=lambda x: "n/a" if isinstance(x,float) and math.isnan(x) else f"{100*x:5.1f}%"
    print(f"{label:12s} {s['mode']:24s} {s['rows']:5d} {s['opt_ms']:9.1f} {s['opt_per']:8.2f} {s['ctrl_ms']:8.1f} {s['dense_ms']:8.1f} {pct(s['cont']):>7s} {s['nviol']:8.2f} {s['margin']:8.2f}")

base=results[0][1]
print("\nvs dense:")
for label,s in results[1:]:
    def r(k):
        b=base[k]; v=s[k]
        return "n/a" if b==0 else f"{v/b:.3f}x ({(v-b)/b*100:+.1f}%)"
    print(f"  {label}: opt={r('opt_ms')}, opt/call={r('opt_per')}, evals={r('evals')}, lbfgs={r('lbfgs_ms')}")
print(f"\nArtifacts: {out}")
PY
