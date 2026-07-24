#!/usr/bin/env bash
# Phase-1 closed-loop A/B: same goal as Phase-0 stationarity study.
# Primary comparison: fast LBFGS dense vs fast LBFGS Bezier depth2 V2.
# Optional third arm: adaptive depth0/depth2 (same objective family).
set -eo pipefail

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
source /opt/ros/noetic/setup.bash
source /root/ws/real_planner/devel/setup.bash
set -u

CFG=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/config/click_real_highspeed.yaml
LOGDIR=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log
OUT=/root/ws/real_planner/src/General-Planner/src/Planner/general_planner/log/phase1_legacy_faststop_ab_$(date +%Y%m%d_%H%M%S)
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
  set_yaml_key lbfgs_fast_phase0_guards_en false
  set_yaml_key convex_hull_subdivision_depth 2
  set_yaml_key convex_hull_cost_version 2
  grep -E "convex_hull_en:|convex_hull_adaptive_en:|convex_hull_alm_en:|lbfgs_fast_en:|lbfgs_fast_phase0_guards_en:|convex_hull_cost_version:|convex_hull_subdivision_depth:" "$CFG"

  rm -f "$LOGDIR/time_consuming_${mode_expect}.csv"
  # Clear any stale mode csv so we never pick the wrong file.
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
    ls -la "$LOGDIR"/time_consuming_*.csv 2>&1 || true
    return 1
  fi
  cp -a "$csv" "$OUT/time_consuming_${label}.csv"
  echo "saved $OUT/time_consuming_${label}.csv ($(wc -l < "$OUT/time_consuming_${label}.csv") lines) from $(basename "$csv")"
  head -1 "$csv" >"$OUT/timing_header.txt"
}

restore_cfg() {
  cp -a "$OUT/click_real_highspeed.yaml.bak" "$CFG"
  echo "restored yaml"
}

trap "stop_stack; restore_cfg" EXIT

# Primary arms requested by user.
run_one fast_dense dense false false
run_one fast_hull_fixed_d2 convex_bezier_v2_d2 true false
# Continuity with Phase-0 three-arm study.
run_one fast_hull_twostage_d2 convex_bezier_v2_twostage_d2 true true

python3 - "$OUT" <<'PY'
import csv, math, sys
from pathlib import Path
out = Path(sys.argv[1])
labels = ["fast_dense", "fast_hull_fixed_d2", "fast_hull_twostage_d2"]

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
    def fsum(k):
        return sum(vals(k))
    def fmean(k):
        v = vals(k)
        return sum(v) / len(v) if v else float("nan")
    def frate(k):
        v = vals(k)
        return (sum(1 for x in v if x > 0.5) / len(v)) if v else float("nan")
    def fmax(k):
        v = vals(k)
        return max(v) if v else float("nan")
    def fmin(k):
        v = vals(k)
        return min(v) if v else float("nan")
    modes = [(r.get("EXP_COST_MODE") or "").strip() for r in rows]
    mode = modes[-1] if modes else ""
    return {
        "rows": len(rows),
        "mode": mode,
        "opt_ms": 1000 * fsum("EXP_TRAJ_OPT"),
        "lbfgs_ms": fsum("EXP_LBFGS_MS"),
        "minco_ms": fsum("EXP_MINCO_EVALUATION_MS"),
        "dense_ms": fsum("EXP_DENSE_INTEGRAL_MS"),
        "ctrl_ms": fsum("EXP_CONTROL_POINT_FUNCTIONAL_MS"),
        "evals": fsum("EXP_EVALUATIONS"),
        "iters": fsum("EXP_LBFGS_ITERATIONS"),
        "evals_per_call": fmean("EXP_EVALUATIONS"),
        "hull_checks": fmean("EXP_HULL_CONTROL_CHECKS_PER_EVAL"),
        "scalar_checks": fmean("EXP_SCALAR_CONSTRAINT_CHECKS"),
        "coarse": fmean("EXP_ADAPTIVE_COARSE_SEGMENTS"),
        "fine": fmean("EXP_ADAPTIVE_FINE_SEGMENTS"),
        "fast_stop_rate": frate("EXP_FAST_STOP"),
        "continuous_rate": frate("EXP_CONTINUOUS_FEASIBLE"),
        "robust_rate": frate("EXP_ROBUSTLY_CERTIFIED"),
        "incumbent_rate": frate("EXP_HAS_CERTIFIED_INCUMBENT"),
        "max_norm_viol": fmax("EXP_MAX_NORMALIZED_VIOLATION"),
        "mean_norm_viol": fmean("EXP_MAX_NORMALIZED_VIOLATION"),
        "min_pos_margin": fmin("EXP_MIN_POSITION_MARGIN"),
        "mean_pos_margin": fmean("EXP_MIN_POSITION_MARGIN"),
        "mean_stationarity": fmean("EXP_STATIONARITY_RESIDUAL"),
        "max_stationarity": fmax("EXP_STATIONARITY_RESIDUAL"),
        "opt_per_call_ms": (1000 * fsum("EXP_TRAJ_OPT") / len(rows)) if rows else float("nan"),
        "lbfgs_per_call_ms": (fsum("EXP_LBFGS_MS") / len(rows)) if rows else float("nan"),
    }

results = []
print("\n=== Phase-1 closed-loop A/B (goal 69.032,1.901,1.500, ~25s) ===")
print("Oracle: adaptive Bezier certificate (Phase-1).")
print("Fast-stop: legacy cost/step/penalty (phase0_guards=false).")
print(f"{'label':24s} {'mode':30s} {'calls':>5s} {'opt_ms':>10s} {'lbfgs_ms':>10s} {'minco_ms':>10s} {'dense_ms':>10s} {'ctrl_ms':>10s} {'evals':>8s} {'e/call':>8s} {'hull':>8s} {'scalar':>8s} {'c/f':>9s}")
for label in labels:
    p = out / f"time_consuming_{label}.csv"
    s = summarize(p)
    results.append((label, s))
    print(
        f"{label:24s} {s['mode']:30s} {s['rows']:5d} {s['opt_ms']:10.1f} "
        f"{s['lbfgs_ms']:10.1f} {s['minco_ms']:10.1f} {s['dense_ms']:10.1f} "
        f"{s['ctrl_ms']:10.1f} {s['evals']:8.0f} {s['evals_per_call']:8.1f} "
        f"{s['hull_checks']:8.1f} {s['scalar_checks']:8.1f} "
        f"{s['coarse']:.1f}/{s['fine']:.1f}"
    )

print("\n=== Primary: fast_dense vs fast_hull_fixed_d2 (depth2 V2) ===")
dense, fixed = results[0][1], results[1][1]

def ratio(a, b, k):
    bv, av = b[k], a[k]
    if bv == 0:
        return "n/a"
    return f"{av/bv:.3f}x ({(av-bv)/bv*100:+.1f}%)"

print(
    f"  fixed_d2 / dense: opt={ratio(fixed, dense, 'opt_ms')}, "
    f"lbfgs={ratio(fixed, dense, 'lbfgs_ms')}, "
    f"evals={ratio(fixed, dense, 'evals')}, "
    f"minco={ratio(fixed, dense, 'minco_ms')}, "
    f"opt/call={ratio(fixed, dense, 'opt_per_call_ms')}"
)
print(
    f"  dense:   cont={100*dense['continuous_rate']:.1f}% robust={100*dense['robust_rate']:.1f}% "
    f"incumb={100*dense['incumbent_rate']:.1f}% mean_nviol={dense['mean_norm_viol']:.4g} "
    f"mean_margin={dense['mean_pos_margin']:.4g}"
)
print(
    f"  depth2:  cont={100*fixed['continuous_rate']:.1f}% robust={100*fixed['robust_rate']:.1f}% "
    f"incumb={100*fixed['incumbent_rate']:.1f}% mean_nviol={fixed['mean_norm_viol']:.4g} "
    f"mean_margin={fixed['mean_pos_margin']:.4g}"
)

print("\n=== Phase-1 certificate quality ===")
print(
    f"{'label':24s} {'fast_stop':>9s} {'cont_feas':>9s} {'robust':>9s} "
    f"{'incumb':>8s} {'max_nviol':>10s} {'mean_nviol':>10s} "
    f"{'min_margin':>10s} {'mean_margin':>11s} {'mean_stat':>10s} {'max_stat':>10s}"
)
for label, s in results:
    def pct(x):
        return "n/a" if isinstance(x, float) and math.isnan(x) else f"{100.0 * x:6.1f}%"
    def num(x):
        return "n/a" if isinstance(x, float) and math.isnan(x) else f"{x:10.4g}"
    print(
        f"{label:24s} {pct(s['fast_stop_rate']):>9s} {pct(s['continuous_rate']):>9s} "
        f"{pct(s['robust_rate']):>9s} {pct(s['incumbent_rate']):>8s} "
        f"{num(s['max_norm_viol']):>10s} {num(s['mean_norm_viol']):>10s} "
        f"{num(s['min_pos_margin']):>10s} {num(s['mean_pos_margin']):>11s} "
        f"{num(s['mean_stationarity']):>10s} {num(s['max_stationarity']):>10s}"
    )

base = results[0][1]
print("\nvs fast_dense:")
for label, s in results[1:]:
    def r(k):
        b = base[k]
        v = s[k]
        return "n/a" if b == 0 else f"{v/b:.3f}x ({(v-b)/b*100:+.1f}%)"
    print(
        f"  {label}: opt={r('opt_ms')}, lbfgs={r('lbfgs_ms')}, "
        f"evals={r('evals')}, minco={r('minco_ms')}"
    )

fixed = results[1][1]
two = results[2][1]
print("\nvs fast_hull_fixed_d2:")

def r2(k):
    b = fixed[k]
    v = two[k]
    return "n/a" if b == 0 else f"{v/b:.3f}x ({(v-b)/b*100:+.1f}%)"

print(
    f"  fast_hull_twostage_d2: opt={r2('opt_ms')}, lbfgs={r2('lbfgs_ms')}, "
    f"evals={r2('evals')}, ctrl={r2('ctrl_ms')}, hull_checks={r2('hull_checks')}"
)

with (out / "phase1_quality_summary.csv").open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow([
        "label", "mode", "calls", "opt_ms", "lbfgs_ms", "evals",
        "opt_per_call_ms", "lbfgs_per_call_ms", "scalar_checks",
        "fast_stop_rate", "continuous_rate", "robust_rate", "incumbent_rate",
        "max_norm_viol", "mean_norm_viol", "min_pos_margin", "mean_pos_margin",
        "mean_stationarity", "max_stationarity",
    ])
    for label, s in results:
        w.writerow([
            label, s["mode"], s["rows"], s["opt_ms"], s["lbfgs_ms"], s["evals"],
            s["opt_per_call_ms"], s["lbfgs_per_call_ms"], s["scalar_checks"],
            s["fast_stop_rate"], s["continuous_rate"], s["robust_rate"],
            s["incumbent_rate"], s["max_norm_viol"], s["mean_norm_viol"],
            s["min_pos_margin"], s["mean_pos_margin"],
            s["mean_stationarity"], s["max_stationarity"],
        ])
print(f"\nArtifacts: {out}")
PY
