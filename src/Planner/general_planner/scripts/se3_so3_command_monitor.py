#!/usr/bin/env python3

import math
import sys
import time
from threading import Lock

import rospy
from rosgraph_msgs.msg import Log
from quadrotor_msgs.msg import PolynomialTrajectory, PositionCommand, SO3Command


def finite(value):
    return math.isfinite(float(value))


def finite_vec3(msg):
    return finite(msg.x) and finite(msg.y) and finite(msg.z)


class SE3SO3CommandMonitor:
    def __init__(self):
        self.so3_topic = rospy.get_param("~so3_topic", "/se3_demo/so3_cmd")
        self.pos_topic = rospy.get_param("~pos_topic", "/se3_demo/pos_cmd")
        self.traj_topic = rospy.get_param("~traj_topic", "/se3_demo/poly_traj")
        self.require_traj = bool(rospy.get_param("~require_traj", True))
        self.report_period = float(rospy.get_param("~report_period", 1.0))
        self.first_traj_timeout = float(rospy.get_param("~first_traj_timeout", 20.0))
        self.so3_after_traj_timeout = float(rospy.get_param("~so3_after_traj_timeout", 5.0))
        self.run_duration = float(rospy.get_param("~run_duration", 0.0))
        self.fail_fast = bool(rospy.get_param("~fail_fast", False))
        self.min_quat_norm = float(rospy.get_param("~min_quat_norm", 0.95))
        self.max_quat_norm = float(rospy.get_param("~max_quat_norm", 1.05))
        self.min_force_norm = float(rospy.get_param("~min_force_norm", 0.1))
        self.max_invalid = int(rospy.get_param("~max_invalid", 0))
        self.require_demo_done = bool(rospy.get_param("~require_demo_done", False))
        self.pass_on_demo_done = bool(rospy.get_param("~pass_on_demo_done", True))
        self.demo_done_pattern = rospy.get_param("~demo_done_pattern", "SE3_WAYPOINT_DEMO_DONE")

        self.lock = Lock()
        self.start_wall = time.monotonic()
        self.last_report_wall = self.start_wall
        self.first_traj_wall = None
        self.first_so3_wall = None
        self.last_so3_wall = None
        self.last_pos_wall = None
        self.traj_count = 0
        self.heartbeat_count = 0
        self.so3_count = 0
        self.pos_count = 0
        self.invalid_count = 0
        self.demo_done = False
        self.exit_code = 0
        self.failed = False

        self.so3_sub = rospy.Subscriber(self.so3_topic, SO3Command, self.so3_cb, queue_size=100)
        self.pos_sub = rospy.Subscriber(self.pos_topic, PositionCommand, self.pos_cb, queue_size=100)
        self.traj_sub = None
        if self.require_traj:
            self.traj_sub = rospy.Subscriber(self.traj_topic, PolynomialTrajectory, self.traj_cb, queue_size=20)
        self.log_sub = rospy.Subscriber("/rosout", Log, self.log_cb, queue_size=1000)
        self.timer = rospy.Timer(rospy.Duration(max(0.1, self.report_period)), self.timer_cb)

        rospy.loginfo(
            "SE3_SO3_MONITOR_START so3=%s pos=%s traj=%s require_traj=%s",
            self.so3_topic,
            self.pos_topic,
            self.traj_topic,
            str(self.require_traj).lower(),
        )

    def fail(self, reason):
        if self.failed:
            return
        self.failed = True
        self.exit_code = 1
        rospy.logerr("SE3_SO3_MONITOR_FAIL reason=%s", reason)
        if self.fail_fast:
            rospy.signal_shutdown(reason)

    def traj_cb(self, msg):
        now = time.monotonic()
        is_position_traj = (
            (msg.type & PolynomialTrajectory.POSITION_TRAJ) != 0 and
            int(msg.piece_num_pos) > 0 and
            int(msg.order_pos) >= 1
        )
        with self.lock:
            if not is_position_traj:
                self.heartbeat_count += 1
                return
            self.traj_count += 1
            if self.first_traj_wall is None:
                self.first_traj_wall = now
                rospy.loginfo("SE3_SO3_MONITOR_TRAJ_RECEIVED")

        coeff_num = int(msg.order_pos) + 1
        piece_num = int(msg.piece_num_pos)
        expected = piece_num * coeff_num
        if (
            len(msg.time_pos) < piece_num or
            len(msg.coef_pos_x) < expected or
            len(msg.coef_pos_y) < expected or
            len(msg.coef_pos_z) < expected
        ):
            self.fail("invalid_polynomial_trajectory_shape")
            return
        if any(float(t) <= 1.0e-6 or not finite(t) for t in msg.time_pos[:piece_num]):
            self.fail("invalid_polynomial_trajectory_time")
            return
        coeffs = (
            list(msg.coef_pos_x[:expected]) +
            list(msg.coef_pos_y[:expected]) +
            list(msg.coef_pos_z[:expected])
        )
        if not all(finite(c) for c in coeffs):
            self.fail("non_finite_polynomial_coefficients")

    def pos_cb(self, msg):
        del msg
        with self.lock:
            self.pos_count += 1
            self.last_pos_wall = time.monotonic()

    def so3_cb(self, msg):
        now = time.monotonic()
        invalid_reason = None
        if not finite_vec3(msg.force):
            invalid_reason = "non_finite_force"
        else:
            q = msg.orientation
            q_norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
            force_norm = math.sqrt(
                msg.force.x * msg.force.x +
                msg.force.y * msg.force.y +
                msg.force.z * msg.force.z
            )
            if not finite(q_norm):
                invalid_reason = "non_finite_quaternion"
            elif q_norm < self.min_quat_norm or q_norm > self.max_quat_norm:
                invalid_reason = "bad_quaternion_norm:{:.6f}".format(q_norm)
            elif not finite(force_norm) or force_norm < self.min_force_norm:
                invalid_reason = "bad_force_norm:{:.6f}".format(force_norm)
            elif not all(finite(v) for v in list(msg.kR) + list(msg.kOm)):
                invalid_reason = "non_finite_gains"

        with self.lock:
            self.so3_count += 1
            self.last_so3_wall = now
            if self.first_so3_wall is None:
                self.first_so3_wall = now
                rospy.loginfo("SE3_SO3_MONITOR_SO3_RECEIVED")
            if invalid_reason is not None:
                self.invalid_count += 1
                rospy.logerr("SE3_SO3_MONITOR_INVALID reason=%s", invalid_reason)
                if self.invalid_count > self.max_invalid:
                    self.fail("too_many_invalid_so3")

    def log_cb(self, msg):
        text = msg.msg
        if self.demo_done_pattern and self.demo_done_pattern in text:
            with self.lock:
                if not self.demo_done:
                    self.demo_done = True
                    rospy.loginfo("SE3_SO3_MONITOR_DEMO_DONE")
        if "SE3_OPT_FAILED" in text or "OPTIMIZER_FAILED" in text:
            self.fail("optimizer_failed_log")

    def timer_cb(self, _event):
        now = time.monotonic()
        with self.lock:
            elapsed = now - self.start_wall
            traj_count = self.traj_count
            heartbeat_count = self.heartbeat_count
            so3_count = self.so3_count
            pos_count = self.pos_count
            invalid_count = self.invalid_count
            demo_done = self.demo_done
            first_traj_wall = self.first_traj_wall
            first_so3_wall = self.first_so3_wall
            last_so3_wall = self.last_so3_wall

        if self.require_traj and traj_count == 0 and elapsed > self.first_traj_timeout:
            self.fail("no_polynomial_trajectory_after_{:.1f}s".format(self.first_traj_timeout))
            return

        if first_traj_wall is not None and so3_count == 0:
            wait = now - first_traj_wall
            if wait > self.so3_after_traj_timeout:
                self.fail("no_so3_command_after_trajectory")
                return

        if first_so3_wall is not None and last_so3_wall is not None and last_so3_wall > first_so3_wall:
            so3_rate = so3_count / max(1.0e-6, last_so3_wall - first_so3_wall)
        else:
            so3_rate = 0.0

        rospy.loginfo(
            "SE3_SO3_MONITOR_STATUS elapsed=%.1fs traj=%d heartbeat=%d pos=%d so3=%d so3_rate=%.1fHz invalid=%d done=%s",
            elapsed,
            traj_count,
            heartbeat_count,
            pos_count,
            so3_count,
            so3_rate,
            invalid_count,
            str(demo_done).lower(),
        )

        success = (
            self.exit_code == 0 and
            (not self.require_traj or traj_count > 0) and
            pos_count > 0 and
            so3_count > 0 and
            invalid_count <= self.max_invalid
        )
        if self.require_demo_done:
            success = success and demo_done

        if self.pass_on_demo_done and self.require_demo_done and success:
            rospy.loginfo("SE3_SO3_MONITOR_PASS")
            rospy.signal_shutdown("demo done")
            return

        if self.run_duration > 0.0 and elapsed >= self.run_duration:
            if success:
                rospy.loginfo("SE3_SO3_MONITOR_PASS")
                rospy.signal_shutdown("run_duration reached")
            else:
                self.fail("run_duration_reached_without_success")


def main():
    rospy.init_node("se3_so3_command_monitor", anonymous=False)
    monitor = SE3SO3CommandMonitor()
    rospy.spin()
    return monitor.exit_code


if __name__ == "__main__":
    sys.exit(main())
