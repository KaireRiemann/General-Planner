#!/usr/bin/env python3
"""Adapt actual Unity car odometry to tracking input; never synthesize motion.

Unity provides base pose in world, twist in the car frame, and uptime stamps.
Convert twist to world, add the same center height used by perception, and
stamp each received sample with ROS time. Do not refresh stopped/old input.
"""
import copy
import json
import math
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String
from tf.transformations import quaternion_matrix


class UnityTargetGroundTruth:
    def __init__(self):
        self.lock = threading.RLock()
        self.timeout = float(rospy.get_param("~input_timeout", 0.5))
        self.center_height = float(rospy.get_param("~target_center_height", 0.7))
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("input_timeout must be positive and finite")
        if not math.isfinite(self.center_height):
            raise ValueError("target_center_height must be finite")
        self.last_stamp = None
        self.last_receipt = None
        self.reason = "waiting_for_ground_truth"
        self.valid = False
        self.odom_pub = rospy.Publisher("~target_odom", Odometry, queue_size=1)
        self.valid_pub = rospy.Publisher("~valid", Bool, queue_size=1, latch=True)
        self.age_pub = rospy.Publisher("~observation_age", Float64, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.valid_pub.publish(False)
        self.sub = rospy.Subscriber("~input", Odometry, self.on_odom,
                                    queue_size=1, tcp_nodelay=True)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)
        rospy.on_shutdown(lambda: self.valid_pub.publish(False))
        rospy.loginfo("[tracking ground truth] actual car pose; center_height=%.3f; timeout=%.3f; ROS reception stamps",
                      self.center_height, self.timeout)

    def on_odom(self, msg):
        with self.lock:
            stamp = msg.header.stamp.to_sec()
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            v, w = msg.twist.twist.linear, msg.twist.twist.angular
            values = [stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                      v.x, v.y, v.z, w.x, w.y, w.z]
            if (msg.header.frame_id != "world" or stamp <= 0 or
                    not all(math.isfinite(x) for x in values) or
                    np.linalg.norm([q.x, q.y, q.z, q.w]) < 1e-6):
                self.valid = False
                self.reason = "invalid_ground_truth"
                self.publish_status()
                return
            if self.last_stamp is not None and stamp <= self.last_stamp:
                if stamp < self.last_stamp:
                    self.last_stamp = stamp
                    self.last_receipt = None
                    self.valid = False
                    self.reason = "source_clock_reset"
                else:
                    self.reason = "duplicate_source_stamp"
                self.publish_status()
                return
            rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            world_v = rotation @ np.array([v.x, v.y, v.z])
            world_w = rotation @ np.array([w.x, w.y, w.z])
            out = copy.deepcopy(msg)
            out.header.stamp = rospy.Time.now()
            # Tracking consumes world velocity, as the visual estimator does.
            out.child_frame_id = ""
            out.pose.pose.position.z += self.center_height
            out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = world_v
            out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = world_w
            self.last_stamp = stamp
            self.last_receipt = time.monotonic()
            self.valid = True
            self.reason = "ground_truth"
            self.publish_status()
            self.odom_pub.publish(out)

    def publish_status(self):
        age = float("inf") if self.last_receipt is None else time.monotonic() - self.last_receipt
        if age > self.timeout:
            self.valid = False
            if self.reason in ("ground_truth", "duplicate_source_stamp"):
                self.reason = "ground_truth_timeout"
        self.valid_pub.publish(self.valid)
        self.age_pub.publish(age)
        self.status_pub.publish(json.dumps(dict(
            state="tracking" if self.valid else "lost", valid=self.valid,
            source="unity_ground_truth", reason=self.reason,
            range_method="ground_truth", observation_age=age if math.isfinite(age) else None,
            observation_stamp=self.last_stamp, clock="sensor", output_clock="ros",
            validity_reason="valid" if self.valid else self.reason)))

    def tick(self, _):
        with self.lock:
            self.publish_status()


if __name__ == "__main__":
    rospy.init_node("unity_target_ground_truth")
    UnityTargetGroundTruth()
    rospy.spin()
