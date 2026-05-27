#!/usr/bin/env python3
"""Export planner trajectory topics from a ROS1 bag to PlotJuggler-friendly CSV."""

import argparse
import csv
import math
import os
import sys

try:
    import rosbag
except ImportError as exc:
    print(
        "rosbag is not available. Source ROS first, for example:\n"
        "  source /opt/ros/noetic/setup.bash\n"
        "  source /root/ws/real_exp/devel/setup.bash",
        file=sys.stderr,
    )
    raise SystemExit(1) from exc


POSITION_TRAJ = 2
YAW_TRAJ = 4


def stamp_to_sec(stamp):
    return stamp.to_sec() if hasattr(stamp, "to_sec") else float(stamp)


def header_time(msg, fallback_time):
    if hasattr(msg, "header") and msg.header.stamp.to_sec() > 0.0:
        return msg.header.stamp.to_sec()
    return stamp_to_sec(fallback_time)


def norm3(vec):
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)


def poly_value(coeffs, local_t, derivative=0):
    degree = len(coeffs) - 1
    value = 0.0
    for idx, coeff in enumerate(coeffs):
        power = degree - idx
        if power < derivative:
            continue
        factor = 1.0
        for k in range(derivative):
            factor *= power - k
        value += coeff * factor * (local_t ** (power - derivative))
    return value


def get_coeff(msg, axis, piece_idx, order):
    coeff_num = order + 1
    start = piece_idx * coeff_num
    end = start + coeff_num
    if axis == "x":
        return list(msg.coef_pos_x[start:end])
    if axis == "y":
        return list(msg.coef_pos_y[start:end])
    if axis == "z":
        return list(msg.coef_pos_z[start:end])
    if axis == "yaw":
        return list(msg.coef_yaw[start:end])
    raise ValueError(axis)


def locate_piece(durations, traj_t):
    elapsed = 0.0
    for idx, duration in enumerate(durations):
        end = elapsed + duration
        if traj_t <= end or idx == len(durations) - 1:
            return idx, max(0.0, min(duration, traj_t - elapsed))
        elapsed = end
    return len(durations) - 1, durations[-1]


def write_position_command_csv(bag, topic, output_path):
    count = 0
    with open(output_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "time",
                "bag_time",
                "trajectory_id",
                "trajectory_flag",
                "pos_x",
                "pos_y",
                "pos_z",
                "vel_x",
                "vel_y",
                "vel_z",
                "acc_x",
                "acc_y",
                "acc_z",
                "jerk_x",
                "jerk_y",
                "jerk_z",
                "yaw",
                "yaw_dot",
                "vel_norm",
                "acc_norm",
                "att_roll",
                "att_pitch",
                "att_yaw",
                "omega_x",
                "omega_y",
                "omega_z",
                "thrust_z",
            ]
        )
        for _, msg, bag_time in bag.read_messages(topics=[topic]):
            count += 1
            writer.writerow(
                [
                    header_time(msg, bag_time),
                    stamp_to_sec(bag_time),
                    getattr(msg, "trajectory_id", 0),
                    getattr(msg, "trajectory_flag", 0),
                    msg.position.x,
                    msg.position.y,
                    msg.position.z,
                    msg.velocity.x,
                    msg.velocity.y,
                    msg.velocity.z,
                    msg.acceleration.x,
                    msg.acceleration.y,
                    msg.acceleration.z,
                    msg.jerk.x,
                    msg.jerk.y,
                    msg.jerk.z,
                    msg.yaw,
                    msg.yaw_dot,
                    getattr(msg, "vel_norm", 0.0) or norm3(msg.velocity),
                    getattr(msg, "acc_norm", 0.0) or norm3(msg.acceleration),
                    msg.attitude.x,
                    msg.attitude.y,
                    msg.attitude.z,
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                    msg.thrust.z,
                ]
            )
    return count


def write_polynomial_samples_csv(bag, topic, output_path, sample_dt):
    count = 0
    with open(output_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "time",
                "bag_time",
                "trajectory_id",
                "trajectory_local_time",
                "piece_index",
                "piece_local_time",
                "total_duration",
                "pos_x",
                "pos_y",
                "pos_z",
                "vel_x",
                "vel_y",
                "vel_z",
                "acc_x",
                "acc_y",
                "acc_z",
                "jerk_x",
                "jerk_y",
                "jerk_z",
                "yaw",
                "yaw_dot",
                "debug_info",
            ]
        )
        for _, msg, bag_time in bag.read_messages(topics=[topic]):
            if (int(msg.type) & POSITION_TRAJ) == 0:
                continue
            piece_num = int(msg.piece_num_pos)
            order = int(msg.order_pos)
            coeff_num = order + 1
            if piece_num <= 0 or order < 1:
                continue
            if (
                len(msg.time_pos) < piece_num
                or len(msg.coef_pos_x) < piece_num * coeff_num
                or len(msg.coef_pos_y) < piece_num * coeff_num
                or len(msg.coef_pos_z) < piece_num * coeff_num
            ):
                continue

            durations = [float(msg.time_pos[i]) for i in range(piece_num)]
            total_duration = sum(d for d in durations if d > 0.0)
            if total_duration <= 0.0:
                continue

            start_time = msg.start_WT_pos.to_sec()
            if start_time <= 0.0:
                start_time = stamp_to_sec(bag_time)

            steps = max(1, int(math.ceil(total_duration / sample_dt)))
            for step in range(steps + 1):
                traj_t = min(total_duration, step * sample_dt)
                piece_idx, piece_t = locate_piece(durations, traj_t)
                coeff_x = get_coeff(msg, "x", piece_idx, order)
                coeff_y = get_coeff(msg, "y", piece_idx, order)
                coeff_z = get_coeff(msg, "z", piece_idx, order)

                yaw = getattr(msg, "yaw", 0.0)
                yaw_dot = getattr(msg, "yaw_rate", 0.0)
                if int(msg.type) & YAW_TRAJ and msg.piece_num_yaw > 0:
                    yaw_order = int(msg.order_yaw)
                    yaw_durations = [float(msg.time_yaw[i]) for i in range(int(msg.piece_num_yaw))]
                    yaw_piece_idx, yaw_piece_t = locate_piece(yaw_durations, min(traj_t, sum(yaw_durations)))
                    if len(msg.coef_yaw) >= (yaw_piece_idx + 1) * (yaw_order + 1):
                        coeff_yaw = get_coeff(msg, "yaw", yaw_piece_idx, yaw_order)
                        yaw = poly_value(coeff_yaw, yaw_piece_t, 0)
                        yaw_dot = poly_value(coeff_yaw, yaw_piece_t, 1)

                count += 1
                writer.writerow(
                    [
                        start_time + traj_t,
                        stamp_to_sec(bag_time),
                        msg.trajectory_id,
                        traj_t,
                        piece_idx,
                        piece_t,
                        total_duration,
                        poly_value(coeff_x, piece_t, 0),
                        poly_value(coeff_y, piece_t, 0),
                        poly_value(coeff_z, piece_t, 0),
                        poly_value(coeff_x, piece_t, 1),
                        poly_value(coeff_y, piece_t, 1),
                        poly_value(coeff_z, piece_t, 1),
                        poly_value(coeff_x, piece_t, 2),
                        poly_value(coeff_y, piece_t, 2),
                        poly_value(coeff_z, piece_t, 2),
                        poly_value(coeff_x, piece_t, 3),
                        poly_value(coeff_y, piece_t, 3),
                        poly_value(coeff_z, piece_t, 3),
                        yaw,
                        yaw_dot,
                        getattr(msg, "debug_info", ""),
                    ]
                )
    return count


def main():
    parser = argparse.ArgumentParser(
        description="Export /planning/pos_cmd and /planning_cmd/poly_traj from a ROS1 bag."
    )
    parser.add_argument("bag", help="Input .bag file")
    parser.add_argument(
        "-o",
        "--output-dir",
        default=None,
        help="Output directory. Defaults to the bag directory.",
    )
    parser.add_argument("--cmd-topic", default="/planning/pos_cmd")
    parser.add_argument("--poly-topic", default="/planning_cmd/poly_traj")
    parser.add_argument("--dt", type=float, default=0.01, help="Sampling interval for polynomial trajectories")
    args = parser.parse_args()

    if args.dt <= 0.0:
        parser.error("--dt must be positive")

    bag_path = os.path.abspath(args.bag)
    output_dir = os.path.abspath(args.output_dir or os.path.dirname(bag_path))
    os.makedirs(output_dir, exist_ok=True)

    base = os.path.splitext(os.path.basename(bag_path))[0]
    cmd_csv = os.path.join(output_dir, base + "_pos_cmd.csv")
    poly_csv = os.path.join(output_dir, base + "_poly_traj_samples.csv")

    with rosbag.Bag(bag_path, "r") as bag:
        cmd_count = write_position_command_csv(bag, args.cmd_topic, cmd_csv)
        poly_count = write_polynomial_samples_csv(bag, args.poly_topic, poly_csv, args.dt)

    print(f"[export_planner_bag_csv] wrote {cmd_count} command rows: {cmd_csv}")
    print(f"[export_planner_bag_csv] wrote {poly_count} polynomial sample rows: {poly_csv}")
    if cmd_count == 0:
        print(f"[export_planner_bag_csv] no messages found on {args.cmd_topic}", file=sys.stderr)
    if poly_count == 0:
        print(f"[export_planner_bag_csv] no full polynomial trajectory messages found on {args.poly_topic}", file=sys.stderr)


if __name__ == "__main__":
    main()
