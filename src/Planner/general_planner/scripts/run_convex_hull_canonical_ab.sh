#!/usr/bin/env bash
# Canonical Bezier hull A/B: dense baseline vs single Stage-A→oracle→Phase-2 path.
set -eo pipefail

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/devel/setup.bash
set -u

CFG=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/config/click_real_highspeed.yaml
LOGDIR=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log
OUT=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log/hull_canonical_ab_$(date +%Y%m%d_%H%M%S)
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
  set_yaml_key convex_hull_phase2_corrector_en true
  set_yaml_key convex_hull_stage_a_seed_refine_en false
  set_yaml_key convex_hull_adaptive_en true
  set_yaml_key lbfgs_fast_en true
  set_yaml_key lbfgs_fast_phase0_guards_en false
  set_yaml_key convex_hull_subdivision_depth 2
  set_yaml_key convex_hull_cost_version 2
  grep -E "convex_hull_en:|convex_hull_phase2_corrector_en:|convex_hull_stage_a_seed_refine_en:|lbfgs_fast_en:" "$CFG"

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
  echo "saved $OUT/time_consuming_${label}.csv ($(wc -l < "$OUT/time_consuming_${label}.csv") lines) from $(basename "$csv")"
}

restore_cfg() {
  cp -a "$OUT/click_real_highspeed.yaml.bak" "$CFG"
  echo "restored yaml"
}

trap "stop_stack; restore_cfg" EXIT

run_one fast_dense dense false
run_one canonical_phase2 convex_bezier_v2_phase2_d2 true

python3 - "$OUT" <<'PY'
import csv, math, sys
from pathlib import Path
out = Path(sys.argv[1])
labels = ["fast_dense", "canonical_phase2"]

def summarize(path):
    rows = list(csv.DictReader(path.open()))
    def vals(k):
        outv = []
        for r in rows:
            s = (r.get(k) or "").strip()
            if s == "":
                continue
            try:
                outv.append(float(s))
            except ValueError:
                pass
        return outv
    def fsum(k): return sum(vals(k))
    def fmean(k):
        v = vals(k)
        return sum(v)/len(v) if v else float("nan")
    def frate(k):
        v = vals(k)
        return (sum(1 for x in v if x > 0.5)/len(v)) if v else float("nan")
    return {
        "rows": len(rows),
        "mode": (rows[-1].get("EXP_COST_MODE") or "").strip() if rows else "",
        "opt_ms": 1000*fsum("EXP_TRAJ_OPT"),
        "lbfgs_ms": fsum("EXP_LBFGS_MS"),
        "ctrl_ms": fsum("EXP_CONTROL_POINT_FUNCTIONAL_MS"),
        "evals": fsum("EXP_EVALUATIONS"),
        "e_call": fmean("EXP_EVALUATIONS"),
        "hull": fmean("EXP_HULL_CONTROL_CHECKS_PER_EVAL"),
        "fine": fmean("EXP_ADAPTIVE_FINE_SEGMENTS"),
        "alm_c": fmean("EXP_ALM_CONSTRAINTS"),
        "p2_trig": frate("EXP_PHASE2_TRIGGERED"),
        "p2_pack": fmean("EXP_PHASE2_PACKED_CONSTRAINTS"),
        "fast_stop": frate("EXP_FAST_STOP"),
        "cont": frate("EXP_CONTINUOUS_FEASIBLE"),
        "robust": frate("EXP_ROBUSTLY_CERTIFIED"),
        "mean_nviol": fmean("EXP_MAX_NORMALIZED_VIOLATION"),
        "opt_per": (1000*fsum("EXP_TRAJ_OPT")/len(rows)) if rows else float("nan"),
    }

results=[]
print("\n=== Canonical hull A/B (goal 69.032,1.901,1.500, ~25s) ===")
print(f"{'label':22s} {'mode':28s} {'calls':>5s} {'opt_ms':>9s} {'lbfgs':>9s} {'e/call':>7s} {'fine':>5s} {'alm_c':>6s}")
for label in labels:
    s=summarize(out/f"time_consuming_{label}.csv")
    results.append((label,s))
    print(f"{label:22s} {s['mode']:28s} {s['rows']:5d} {s['opt_ms']:9.1f} {s['lbfgs_ms']:9.1f} {s['e_call']:7.1f} {s['fine']:5.2f} {s['alm_c']:6.1f}")

print("\n=== Quality ===")
print(f"{'label':22s} {'fast%':>7s} {'cont%':>7s} {'rob%':>7s} {'p2%':>7s} {'p2_c':>6s} {'nviol':>10s} {'opt/call':>8s}")
for label,s in results:
    def pct(x):
        return "n/a" if isinstance(x,float) and math.isnan(x) else f"{100*x:5.1f}%"
    print(f"{label:22s} {pct(s['fast_stop']):>7s} {pct(s['cont']):>7s} {pct(s['robust']):>7s} {pct(s['p2_trig']):>7s} {s['p2_pack']:6.1f} {s['mean_nviol']:10.4g} {s['opt_per']:8.2f}")

base=results[0][1]
print("\nvs fast_dense:")
for label,s in results[1:]:
    def r(k):
        b=base[k]; v=s[k]
        return "n/a" if b==0 else f"{v/b:.3f}x ({(v-b)/b*100:+.1f}%)"
    print(f"  {label}: opt={r('opt_ms')}, lbfgs={r('lbfgs_ms')}, evals={r('evals')}, opt/call={r('opt_per')}")

print(f"\nArtifacts: {out}")
PY
