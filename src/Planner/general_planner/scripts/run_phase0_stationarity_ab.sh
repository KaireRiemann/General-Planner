#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/devel/setup.bash

CFG=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/config/click_real_highspeed.yaml
LOGDIR=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log
OUT=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log/hull_ab_manual_$(date +%Y%m%d_%H%M%S)
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
  local label="$1" mode_expect="$2" hull="$3" adaptive="$4"
  echo "===== $label ====="
  set_yaml_key convex_hull_en "$hull"
  set_yaml_key convex_hull_adaptive_en "$adaptive"
  set_yaml_key convex_hull_alm_en false
  set_yaml_key lbfgs_fast_en true
  set_yaml_key convex_hull_subdivision_depth 2
  set_yaml_key convex_hull_cost_version 2
  grep -E "convex_hull_en:|convex_hull_adaptive_en:|convex_hull_alm_en:|lbfgs_fast_en:" "$CFG"

  rm -f "$LOGDIR/time_consuming_${mode_expect}.csv"

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
  echo "saved $OUT/time_consuming_${label}.csv ($(wc -l < "$OUT/time_consuming_${label}.csv") lines)"
}

restore_cfg() {
  cp -a "$OUT/click_real_highspeed.yaml.bak" "$CFG"
  echo "restored yaml"
}

trap "stop_stack; restore_cfg" EXIT

run_one fast_dense dense false false
run_one fast_hull_fixed_d2 convex_bezier_v2_d2 true false
run_one fast_hull_twostage_d2 convex_bezier_v2_twostage_d2 true true

python3 - "$OUT" <<'PY'
import csv, sys
from pathlib import Path
out = Path(sys.argv[1])
labels = ["fast_dense", "fast_hull_fixed_d2", "fast_hull_twostage_d2"]

def summarize(path):
    rows = list(csv.DictReader(path.open()))
    def fsum(k):
        return sum(float(r[k]) for r in rows if (r.get(k) or "").strip()!="")
    def fmean(k):
        vals=[float(r[k]) for r in rows if (r.get(k) or "").strip()!=""]
        return sum(vals)/len(vals) if vals else float("nan")
    return {
        "rows": len(rows),
        "mode": (rows[-1].get("EXP_COST_MODE") or "").strip() if rows else "",
        "opt_ms": 1000*fsum("EXP_TRAJ_OPT"),
        "lbfgs_ms": fsum("EXP_LBFGS_MS"),
        "minco_ms": fsum("EXP_MINCO_EVALUATION_MS"),
        "dense_ms": fsum("EXP_DENSE_INTEGRAL_MS"),
        "ctrl_ms": fsum("EXP_CONTROL_POINT_FUNCTIONAL_MS"),
        "evals": fsum("EXP_EVALUATIONS"),
        "iters": fsum("EXP_LBFGS_ITERATIONS"),
        "evals_per_call": fmean("EXP_EVALUATIONS"),
        "hull_checks": fmean("EXP_HULL_CONTROL_CHECKS_PER_EVAL"),
        "coarse": fmean("EXP_ADAPTIVE_COARSE_SEGMENTS"),
        "fine": fmean("EXP_ADAPTIVE_FINE_SEGMENTS"),
    }

results=[]
print("\n=== Closed-loop A/B (goal 69.032,1.901,1.500, ~25s) ===")
print(f"{'label':24s} {'mode':30s} {'calls':>5s} {'opt_ms':>10s} {'lbfgs_ms':>10s} {'minco_ms':>10s} {'dense_ms':>10s} {'ctrl_ms':>10s} {'evals':>8s} {'e/call':>8s} {'hull':>8s} {'c/f':>9s}")
for label in labels:
    p = out/f"time_consuming_{label}.csv"
    s = summarize(p)
    results.append((label,s))
    print(f"{label:24s} {s['mode']:30s} {s['rows']:5d} {s['opt_ms']:10.1f} {s['lbfgs_ms']:10.1f} {s['minco_ms']:10.1f} {s['dense_ms']:10.1f} {s['ctrl_ms']:10.1f} {s['evals']:8.0f} {s['evals_per_call']:8.1f} {s['hull_checks']:8.1f} {s['coarse']:.1f}/{s['fine']:.1f}")

base=results[0][1]
print("\nvs fast_dense:")
for label,s in results[1:]:
    def r(k):
        b=base[k]; v=s[k]
        return "n/a" if b==0 else f"{v/b:.3f}x ({(v-b)/b*100:+.1f}%)"
    print(f"  {label}: opt={r('opt_ms')}, lbfgs={r('lbfgs_ms')}, evals={r('evals')}, minco={r('minco_ms')}")

fixed=results[1][1]; two=results[2][1]
print("\nvs fast_hull_fixed_d2:")
def r2(k):
    b=fixed[k]; v=two[k]
    return "n/a" if b==0 else f"{v/b:.3f}x ({(v-b)/b*100:+.1f}%)"
print(f"  fast_hull_twostage_d2: opt={r2('opt_ms')}, lbfgs={r2('lbfgs_ms')}, evals={r2('evals')}, ctrl={r2('ctrl_ms')}, hull_checks={r2('hull_checks')}")
print(f"\nArtifacts: {out}")
PY
