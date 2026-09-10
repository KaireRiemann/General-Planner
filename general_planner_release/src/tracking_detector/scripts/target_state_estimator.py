#!/usr/bin/env python3
"""Timestamped target filter. Sensor-clock state; ROS-clock output and quality."""
import bisect
import json
import math
import struct
import threading
import time
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, String, Float64
from tracking_detector.msg import BoundingBoxes


class ConstantVelocityFilter:
    def __init__(self, acceleration_noise=2.0, gate=16.27):
        self.noise, self.gate = acceleration_noise, gate
        self.x = None
        self.P = None
        self.stamp = None

    def predicted(self, dt):
        dt = max(0.0, float(dt))
        A = np.eye(6)
        A[:3, 3:] = np.eye(3) * dt
        B = np.vstack((np.eye(3) * dt * dt / 2, np.eye(3) * dt))
        return A @ self.x, A @ self.P @ A.T + B @ B.T * self.noise ** 2

    def update(self, position, variance, stamp):
        z = np.asarray(position)
        if not np.isfinite(z).all() or not np.isfinite(variance) or variance <= 0:
            return False
        if self.x is None:
            self.x = np.r_[z, np.zeros(3)]
            self.P = np.diag([variance] * 3 + [4.0] * 3)
            self.stamp = stamp
            return True
        if stamp <= self.stamp:
            return False
        x, P = self.predicted(stamp - self.stamp)
        R = np.eye(3) * variance
        innovation = z - x[:3]
        S = P[:3, :3] + R
        if innovation @ np.linalg.solve(S, innovation) > self.gate:
            return False
        K = np.linalg.solve(S, P[:3, :]).T
        self.x = x + K @ innovation
        # Joseph form preserves covariance symmetry and positive definiteness.
        IKH = np.eye(6)
        IKH[:, :3] -= K
        self.P = IKH @ P @ IKH.T + K @ R @ K.T
        self.stamp = stamp
        return True


def rotation(q):
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-6:
        raise ValueError("invalid odometry quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def interpolate_pose(samples, stamp, tolerance):
    if not samples:
        raise ValueError("no odometry")
    times = [s[0] for s in samples]
    i = bisect.bisect_left(times, stamp)
    if i == 0 or i == len(times):
        s = samples[0 if i == 0 else -1]
        if abs(s[0] - stamp) > tolerance:
            raise ValueError("odometry timestamp mismatch")
        return s[1], rotation(s[2])
    a, b = samples[i-1], samples[i]
    if b[0] - a[0] > 2 * tolerance:
        raise ValueError("odometry interpolation gap")
    u = (stamp-a[0])/(b[0]-a[0])
    qa, qb = a[2].copy(), b[2].copy()
    qa /= np.linalg.norm(qa)
    qb /= np.linalg.norm(qb)
    dot = np.dot(qa, qb)
    if dot < 0:
        qb, dot = -qb, -dot
    if dot > 0.9995:
        q = qa + u * (qb-qa)
    else:
        angle = math.acos(np.clip(dot, -1, 1))
        q = (math.sin((1-u)*angle)*qa + math.sin(u*angle)*qb)/math.sin(angle)
    return a[1]*(1-u)+b[1]*u, rotation(q)


def ground_point(u, v, K, camera_p, camera_R, ground_z):
    ray = camera_R @ np.array([(u-K[2])/K[0], (v-K[3])/K[1], 1.0])
    if ray[2] >= -0.03:
        raise ValueError("ground ray near/above horizon")
    distance = (ground_z-camera_p[2])/ray[2]
    if distance <= 0:
        raise ValueError("ground is behind camera")
    return camera_p + distance*ray


class TargetEstimator:
    def __init__(self):
        param = lambda n, d: rospy.get_param("~"+n, d)
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.odom = deque(maxlen=600)
        self.depth = deque(maxlen=20)
        self.pending = deque(maxlen=3)
        self.info = None
        self.last_odom_receipt = 0.0
        self.filter = ConstantVelocityFilter()
        self.last_bbox_stamp = None
        self.last_accept_wall = 0.0
        self.count = 0
        self.track_id = 0
        self.reason = "waiting_for_observation"
        self.method = "none"
        self.label = param("target_label", "car")
        self.output_frame = param("output_frame", "world")
        self.width, self.height = param("bbox_width", 640), param("bbox_height", 480)
        self.Rcb = np.array(param("cam2body_R", [0,0,1,-1,0,0,0,-1,0])).reshape(3,3)
        self.pcb = np.array(param("cam2body_p", [0,0,0.1]))
        if not np.allclose(self.Rcb.T @ self.Rcb, np.eye(3), atol=1e-5) or np.linalg.det(self.Rcb) < 0.99:
            raise ValueError("cam2body_R must be a proper rotation")
        self.ground_z = float(param("ground_z", 0.0))
        self.center_height = float(param("target_center_height", 0.7))
        self.range_method = param("range_method", "ground_plane")
        if self.range_method not in ("ground_plane", "depth", "auto", "known_height"):
            raise ValueError("invalid range_method")
        self.depth_registered = bool(param("depth_registered", False))
        if self.range_method == "depth" and not self.depth_registered:
            raise ValueError("depth mode requires depth_registered=true and aligned metric depth")
        self.depth_scale = float(param("depth_uint16_scale", 0.001))
        if self.depth_scale <= 0:
            raise ValueError("depth_uint16_scale must be positive")
        self.object_height = float(param("object_height", 1.4))
        self.sync_tolerance = float(param("sync_tolerance", 0.15))
        self.depth_tolerance = float(param("depth_tolerance", 0.08))
        self.max_age = float(param("max_observation_age", 1.0))
        self.max_receipt_gap = float(param("max_detection_gap", 0.45))
        self.was_valid = False
        self.fresh_age = float(param("fresh_observation_age", 0.25))
        self.confirmations = max(2, int(param("confirmations", 3)))
        self.max_range = float(param("max_range", 30.0))
        self.max_position_std = float(param("max_position_std", 2.0))
        self.odom_pub = rospy.Publisher("~target_odom", Odometry, queue_size=1)
        self.raw_pub = rospy.Publisher("~yolo_odom", Odometry, queue_size=1)
        self.valid_pub = rospy.Publisher("~valid", Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.age_pub = rospy.Publisher("~observation_age", Float64, queue_size=1)
        self.valid_pub.publish(False)
        rospy.Subscriber("~odom", Odometry, self.on_odom, queue_size=100)
        rospy.Subscriber("~camera_info", CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber("~yolo", BoundingBoxes, self.on_bbox, queue_size=2)
        if self.depth_registered and self.range_method in ("depth", "auto"):
            cls = CompressedImage if param("depth_compressed", True) else Image
            rospy.Subscriber("~depth", cls, self.on_depth, queue_size=2,
                             buff_size=8*1024*1024)
        rospy.on_shutdown(lambda: self.valid_pub.publish(False))
        rospy.Timer(rospy.Duration(0.05), self.tick)

    def reset(self, reason):
        self.filter = ConstantVelocityFilter()
        self.count = 0
        self.reason = reason

    def on_info(self, msg):
        if msg.width and msg.height and msg.K[0] > 0 and msg.K[4] > 0:
            with self.lock:
                self.info = msg

    def on_odom(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.output_frame:
            rospy.logwarn_throttle(2.0, "target estimator odom frame differs from output_frame; configure a transform upstream")
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = np.array([p.x,p.y,p.z])
        quat = np.array([q.x,q.y,q.z,q.w])
        if not np.isfinite(pose).all():
            return
        try:
            rotation(quat)
        except ValueError:
            return
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0:
            return
        with self.lock:
            if self.odom and stamp < self.odom[-1][0] - 0.5:
                self.odom.clear()
                self.pending.clear()
                self.depth.clear()
                self.last_bbox_stamp = None
                self.reset("sensor_clock_reset")
            if self.odom and stamp <= self.odom[-1][0]:
                return
            self.odom.append((stamp, pose, quat))
            self.last_odom_receipt = time.monotonic()

    def on_depth(self, msg):
        try:
            if isinstance(msg, CompressedImage):
                # Support lossless 16-bit PNG. Reject display/JPEG depth.
                data = bytes(msg.data)
                offset = data.find(b"\x89PNG")
                if offset < 0:
                    raise ValueError("depth must be lossless PNG")
                depth = cv2.imdecode(np.frombuffer(data[offset:],np.uint8), cv2.IMREAD_UNCHANGED)
                if depth is None or depth.dtype != np.uint16 or depth.ndim != 2:
                    raise ValueError("compressed depth must contain uint16 metric values")
            else:
                if msg.encoding not in ("16UC1", "32FC1"):
                    raise ValueError("depth encoding must be 16UC1/32FC1")
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if isinstance(msg, CompressedImage) and "32FC1" in msg.format:
                if "compressedDepth" not in msg.format or offset != 12:
                    raise ValueError("unknown inverse-depth transport")
                _, A, B = struct.unpack("<iff", data[:12])
                if not math.isfinite(A) or not math.isfinite(B) or A <= 0:
                    raise ValueError("invalid inverse-depth calibration")
                raw = depth.astype(float)
                depth = np.divide(A, raw-B, out=np.full(raw.shape,np.nan),
                                  where=(raw>0)&(raw>B))
            else:
                depth = depth.astype(float) * (self.depth_scale if depth.dtype == np.uint16 else 1.0)
            with self.lock:
                self.depth.append((msg.header.stamp.to_sec(), depth))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Rejected depth: %s", e)

    def on_bbox(self, msg):
        with self.lock:
            self.pending.append(msg)

    def sensor_now(self):
        return self.odom[-1][0] + max(0.0, time.monotonic()-self.last_odom_receipt)

    def measure(self, box, stamp, p, R):
        info = self.info
        if info is None:
            raise ValueError("waiting_for_camera_info")
        if any(abs(d) > 1e-9 for d in info.D):
            raise ValueError("rectified_image_required_for_pinhole_ranging")
        K = [info.K[0]*self.width/info.width, info.K[4]*self.height/info.height,
             info.K[2]*self.width/info.width, info.K[5]*self.height/info.height]
        u, v = (box.xmin+box.xmax)*0.5, (box.ymin+box.ymax)*0.5
        cp, cR = p+R@self.pcb, R@self.Rcb
        method = self.range_method
        if method in ("depth", "auto") and self.depth_registered and self.depth:
            ds, depth = min(self.depth, key=lambda s: abs(s[0]-stamp))
            if abs(ds-stamp) <= self.depth_tolerance:
                # Central region avoids border/background contamination.
                sx, sy = depth.shape[1]/self.width, depth.shape[0]/self.height
                w, h = (box.xmax-box.xmin)*0.25, (box.ymax-box.ymin)*0.25
                roi = depth[max(0,int((v-h)*sy)):min(depth.shape[0],int((v+h)*sy)+1),
                            max(0,int((u-w)*sx)):min(depth.shape[1],int((u+w)*sx)+1)]
                values = roi[np.isfinite(roi) & (roi>0.2) & (roi<self.max_range)]
                if len(values) >= 20:
                    d = np.percentile(values, 35)
                    foreground = values[abs(values-d) < max(0.15, 0.05*d)]
                    if len(foreground) >= 10:
                        d = float(np.median(foreground))
                        point = cp+cR@np.array([(u-K[2])*d/K[0], (v-K[3])*d/K[1], d])
                        return point, max(0.05, 0.01*d)**2, "depth"
        if method == "depth":
            raise ValueError("no_valid_registered_depth")
        if method in ("auto", "ground_plane"):
            if box.ymax >= self.height-3:
                raise ValueError("ground_contact_clipped")
            point = ground_point(u, box.ymax, K, cp, cR, self.ground_z)
            distance = np.linalg.norm(point-cp)
            point[2] += self.center_height
            return point, (0.08+0.015*distance*distance)**2, "ground_plane"
        if box.ymin <= 2 or box.ymax >= self.height-2 or box.ymax-box.ymin < 5:
            raise ValueError("height_bbox_clipped")
        d = self.object_height*K[1]/(box.ymax-box.ymin)
        return cp+cR@np.array([(u-K[2])*d/K[0],(v-K[3])*d/K[1],d]), (0.15+0.15*d)**2, "known_height"

    def process(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0 or (self.last_bbox_stamp is not None and stamp <= self.last_bbox_stamp):
            self.reason = "out_of_order_bbox"
            return
        self.last_bbox_stamp = stamp
        if not self.odom or self.sensor_now()-stamp > self.max_age or stamp > self.sensor_now()+self.sync_tolerance:
            self.reason = "stale_or_unsynchronized_bbox"
            return
        stale_filter = self.filter.x is not None and stamp-self.filter.stamp > self.max_age
        try:
            p, R = interpolate_pose(self.odom, stamp, self.sync_tolerance)
        except ValueError as e:
            self.reason = str(e)
            return
        candidates = []
        self.reason = "no_matching_detection"
        for box in msg.bounding_boxes:
            if box.Class != self.label or box.xmax <= box.xmin or box.ymax <= box.ymin:
                continue
            try:
                z, variance, method = self.measure(box, stamp, p, R)
                if not np.isfinite(z).all() or np.linalg.norm(z-p) > self.max_range:
                    continue
                if self.filter.x is not None and not stale_filter:
                    x, P = self.filter.predicted(max(0, stamp-self.filter.stamp))
                    delta = z-x[:3]
                    score = delta@np.linalg.solve(P[:3,:3]+np.eye(3)*variance,delta)
                    if score > self.filter.gate:
                        self.reason = "association_gate_rejected"
                        continue
                else:
                    score = -box.probability
                candidates.append((score, z, variance, method))
            except ValueError as e:
                self.reason = str(e)
        if not candidates:
            if self.count < self.confirmations:
                self.count = 0
            if not msg.bounding_boxes:
                self.reason = "no_detection"
            return
        _, z, variance, method = min(candidates, key=lambda c:c[0])
        if stale_filter:
            self.reset("reacquiring")
        if self.filter.update(z, variance, stamp):
            self.count += 1
            if self.count == self.confirmations:
                self.track_id += 1
            self.last_accept_wall = time.monotonic()
            self.reason, self.method = "accepted", method
            raw = self.make_odom(z, np.zeros(3), np.eye(6)*variance, rospy.Time.now())
            self.raw_pub.publish(raw)
        else:
            self.reason = "innovation_rejected"

    def make_odom(self, p, v, covariance, stamp):
        m = Odometry()
        m.header.stamp, m.header.frame_id = stamp, self.output_frame
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = p
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = v
        yaw = math.atan2(v[1],v[0]) if np.linalg.norm(v[:2]) > 0.2 else 0.0
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        for i in range(3):
            for j in range(3):
                m.pose.covariance[i*6+j] = covariance[i,j]
                m.twist.covariance[i*6+j] = covariance[i+3,j+3]
        for i in range(3,6):
            m.pose.covariance[i*6+i] = 1e3  # Heading is derived, not observed.
        return m

    def tick(self, _):
        with self.lock:
            while self.pending:
                self.process(self.pending.popleft())
            now = rospy.Time.now()
            age = max(0.0,self.sensor_now()-self.filter.stamp) if self.filter.x is not None and self.odom else float("inf")
            receipt_age = time.monotonic()-self.last_accept_wall
            validity_reason = "waiting_for_observation"
            valid = False
            state = "lost" if self.track_id else "acquiring"
            x = P = None
            if self.filter.x is not None:
                if age > self.max_age or receipt_age > self.max_receipt_gap or time.monotonic()-self.last_odom_receipt > 0.4:
                    state = "lost"
                    validity_reason = ("observation_timeout" if age > self.max_age else
                                       "detection_gap" if receipt_age > self.max_receipt_gap else "odometry_timeout")
                elif self.count < self.confirmations:
                    state = "acquiring"
                    validity_reason = "confirming_observations"
                else:
                    x,P = self.filter.predicted(age)
                    valid = bool(np.max(np.linalg.eigvalsh(P[:3,:3])) <= self.max_position_std**2)
                    state = ("tracking" if receipt_age<=self.fresh_age else "coasting") if valid else "lost"
                    validity_reason = "valid" if valid else "position_uncertainty"
            # A declared loss requires fresh confirmation before flight resumes.
            if self.was_valid and not valid:
                self.count = 0
            self.was_valid = valid
            self.valid_pub.publish(valid)
            self.age_pub.publish(age)
            self.status_pub.publish(json.dumps(dict(state=state, valid=valid, track_id=self.track_id,
                observation_age=age if math.isfinite(age) else None,
                observation_stamp=self.filter.stamp, clock="sensor", output_clock="ros",
                range_method=self.method, reason=self.reason,
                validity_reason=validity_reason,
                detection_gap=receipt_age if self.last_accept_wall else None,
                position_std=float(np.sqrt(np.max(np.linalg.eigvalsh(P[:3,:3])))) if P is not None else None)))
            if valid:
                self.odom_pub.publish(self.make_odom(x[:3],x[3:],P,now))


if __name__ == "__main__":
    rospy.init_node("target_state_estimator")
    TargetEstimator()
    rospy.spin()
