#!/usr/bin/env python3

import argparse
import glob
import os

import matplotlib.pyplot as plt
import numpy as np


VECTOR_COLUMNS = {
    "Position": ("posi_x", "posi_y", "posi_z"),
    "Velocity": ("vel_x", "vel_y", "vel_z"),
    "Acceleration": ("acc_x", "acc_y", "acc_z"),
    "Jerk": ("jerk_x", "jerk_y", "jerk_z"),
}


def get_latest_csv(search_dir):
    csv_files = glob.glob(os.path.join(search_dir, "*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found in {search_dir}")
    return max(csv_files, key=os.path.getmtime)


def load_csv(csv_path):
    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=float, encoding=None)
    data = np.atleast_1d(data)
    columns = data.dtype.names
    if columns is None:
        raise ValueError(f"{csv_path} has no CSV header")

    matrix = np.column_stack([data[name] for name in columns])
    data = data[~np.all(matrix == 0.0, axis=1)]
    if len(data) == 0:
        raise ValueError(f"{csv_path} has no non-zero rows")
    return data


def vec(data, columns):
    return np.column_stack([data[name] for name in columns])


def norm_and_diff(values):
    norms = np.linalg.norm(values, axis=1)
    diffs = np.diff(norms)
    if len(diffs) == 0:
        return norms, diffs, 0
    return norms, diffs, int(np.argmax(np.abs(diffs)))


def print_basic_summary(data, csv_path):
    time = data["time"]
    backup = data["backup"].astype(int)
    vel_norm = np.linalg.norm(vec(data, VECTOR_COLUMNS["Velocity"]), axis=1)

    print(f"Using CSV file: {os.path.basename(csv_path)}")
    print(f"Rows: {len(data)}")
    if len(time) > 1:
        dt = np.diff(time)
        print(f"Time range: {time[0]:.6f}s -> {time[-1]:.6f}s, duration {time[-1] - time[0]:.6f}s")
        print(
            "dt: "
            f"mean={np.mean(dt):.6f}s, median={np.median(dt):.6f}s, "
            f"min={np.min(dt):.6f}s, max={np.max(dt):.6f}s"
        )
    print(f"Ratio of backup: {np.mean(backup == 2):.6f}")
    print(f"Average Vel. Norm: {np.mean(vel_norm):.6f}")


def print_norm_summary(data):
    time = data["time"]
    print("\nNorm jumps:")
    for label, columns in VECTOR_COLUMNS.items():
        values = vec(data, columns)
        norms, diffs, idx = norm_and_diff(values)
        max_idx = int(np.argmax(norms))
        print(
            f"{label:12s} max={norms[max_idx]:.6f} at t={time[max_idx]:.6f}s; "
            f"max_norm_diff={abs(diffs[idx]) if len(diffs) else 0.0:.6f} "
            f"at t={time[min(idx + 1, len(time) - 1)]:.6f}s"
        )


def print_consistency_summary(data):
    time = data["time"]
    if len(time) < 2:
        return

    dt = np.diff(time)
    pos = vec(data, VECTOR_COLUMNS["Position"])
    vel = vec(data, VECTOR_COLUMNS["Velocity"])
    acc = vec(data, VECTOR_COLUMNS["Acceleration"])
    jerk = vec(data, VECTOR_COLUMNS["Jerk"])
    yaw = data["yaw"]
    yaw_rate = data["yaw_rate"]

    pos_fd = np.diff(pos, axis=0) / dt[:, None]
    vel_mid = 0.5 * (vel[:-1] + vel[1:])
    pos_res = np.linalg.norm(pos_fd - vel_mid, axis=1)

    vel_fd = np.diff(vel, axis=0) / dt[:, None]
    acc_mid = 0.5 * (acc[:-1] + acc[1:])
    vel_res = np.linalg.norm(vel_fd - acc_mid, axis=1)

    acc_fd = np.diff(acc, axis=0) / dt[:, None]
    jerk_mid = 0.5 * (jerk[:-1] + jerk[1:])
    acc_res = np.linalg.norm(acc_fd - jerk_mid, axis=1)

    yaw_fd = np.diff(yaw) / dt
    yaw_rate_mid = 0.5 * (yaw_rate[:-1] + yaw_rate[1:])
    yaw_res = np.abs(yaw_fd - yaw_rate_mid)

    print("\nFinite-difference consistency:")
    for label, residual in (
        ("position-vs-velocity", pos_res),
        ("velocity-vs-acc", vel_res),
        ("acc-vs-jerk", acc_res),
        ("yaw-vs-yaw_rate", yaw_res),
    ):
        idx = int(np.argmax(residual))
        print(
            f"{label:20s} median={np.median(residual):.6f}, "
            f"p95={np.percentile(residual, 95):.6f}, "
            f"max={residual[idx]:.6f} at {time[idx]:.6f}s -> {time[idx + 1]:.6f}s"
        )

    print("\nTop position command discontinuities:")
    top_count = min(10, len(pos_res))
    for idx in np.argsort(pos_res)[-top_count:][::-1]:
        fd_speed = np.linalg.norm(pos_fd[idx])
        cmd_speed = np.linalg.norm(vel_mid[idx])
        delta = pos[idx + 1] - pos[idx]
        print(
            f"idx {idx:4d}->{idx + 1:<4d} "
            f"t {time[idx]:.6f}->{time[idx + 1]:.6f}s "
            f"pos_delta=[{delta[0]: .4f}, {delta[1]: .4f}, {delta[2]: .4f}] "
            f"fd_speed={fd_speed:.3f}m/s cmd_speed={cmd_speed:.3f}m/s "
            f"residual={pos_res[idx]:.3f}"
        )


def plot_norms(data):
    time = data["time"]
    plot_style = {
        "linestyle": "-",
        "linewidth": 1.2,
    }

    fig, axs = plt.subplots(3, 2, figsize=(12, 12), sharex=True)
    axes = axs.ravel()

    for axis, (label, columns) in zip(axes[:4], VECTOR_COLUMNS.items()):
        values = vec(data, columns)
        norms, diffs, idx = norm_and_diff(values)
        axis.plot(time, norms, label=f"{label} Norm", **plot_style)
        if len(diffs):
            axis.axvline(time[idx + 1], color="red", linestyle="--", label="Max Norm Diff")
        axis.set_title(f"{label} Norm")
        axis.set_ylabel("Norm")
        axis.grid(True)
        axis.legend()

    if len(time) > 1:
        pos = vec(data, VECTOR_COLUMNS["Position"])
        vel = vec(data, VECTOR_COLUMNS["Velocity"])
        dt = np.diff(time)
        pos_fd = np.diff(pos, axis=0) / dt[:, None]
        vel_mid = 0.5 * (vel[:-1] + vel[1:])
        pos_res = np.linalg.norm(pos_fd - vel_mid, axis=1)
        max_idx = int(np.argmax(pos_res))
        axes[4].plot(time[1:], pos_res, label="|diff(position)/dt - velocity|", **plot_style)
        axes[4].axvline(time[max_idx + 1], color="red", linestyle="--", label="Max Position Jump")
        axes[4].set_title("Position/Velocity Consistency")
        axes[4].set_ylabel("m/s")
        axes[4].grid(True)
        axes[4].legend()

    backup_times = time[data["backup"].astype(int) == 2]
    for axis in axes[:5]:
        for backup_time in backup_times:
            axis.axvline(backup_time, color="orange", alpha=0.25)

    axes[5].axis("off")
    if len(time) > 1:
        axes[5].text(0.05, 0.80, f"Rows: {len(data)}", transform=axes[5].transAxes)
        axes[5].text(0.05, 0.68, f"Duration: {time[-1] - time[0]:.3f}s", transform=axes[5].transAxes)
        axes[5].text(0.05, 0.56, f"Backup ratio: {np.mean(data['backup'].astype(int) == 2):.3f}", transform=axes[5].transAxes)

    axes[4].set_xlabel("Time (s)")
    plt.tight_layout()


def plot_components(data):
    time = data["time"]
    plot_style = {
        "linestyle": "-",
        "linewidth": 1.1,
    }

    fig, axs = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    for axis, (label, columns) in zip(axs[:4], VECTOR_COLUMNS.items()):
        values = vec(data, columns)
        axis.plot(time, values[:, 0], label=f"{label} X", color="red", **plot_style)
        axis.plot(time, values[:, 1], label=f"{label} Y", color="green", **plot_style)
        axis.plot(time, values[:, 2], label=f"{label} Z", color="blue", **plot_style)
        axis.set_title(f"{label} Components")
        axis.grid(True)
        axis.legend()

    axs[4].plot(time, data["yaw"], label="Yaw", color="purple", **plot_style)
    axs[4].plot(time, data["yaw_rate"], label="Yaw Rate", color="brown", **plot_style)
    axs[4].set_title("Yaw and Yaw Rate")
    axs[4].set_xlabel("Time (s)")
    axs[4].grid(True)
    axs[4].legend()

    backup_times = time[data["backup"].astype(int) == 2]
    for axis in axs:
        for backup_time in backup_times:
            axis.axvline(backup_time, color="orange", alpha=0.25)

    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser(description="Plot and diagnose General-Planner command CSV logs.")
    parser.add_argument("csv", nargs="?", help="CSV file to read. Defaults to the newest CSV beside this script.")
    parser.add_argument("--no-plot", action="store_true", help="Only print diagnostics.")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = args.csv
    if csv_path is None:
        csv_path = get_latest_csv(script_dir)
    elif not os.path.isabs(csv_path):
        csv_path = os.path.abspath(csv_path)

    data = load_csv(csv_path)
    print_basic_summary(data, csv_path)
    print_norm_summary(data)
    print_consistency_summary(data)

    if not args.no_plot:
        plot_norms(data)
        plot_components(data)
        plt.show()


if __name__ == "__main__":
    main()
