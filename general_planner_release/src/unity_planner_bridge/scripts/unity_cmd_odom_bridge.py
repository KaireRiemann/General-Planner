#!/usr/bin/env python3
"""Send planner commands to Unity and forward actual Unity odometry back.

The 100 Hz timer drives Unity only. Planner feedback is published exactly once
per /unity_odom sample, never synthesized from commands or repeated on timeout.
Unity feedback and clouds are stamped with ROS reception time because their
source stamps may use Unity uptime. This is arrival-time synchronization, not
a reconstruction of sensor acquisition time.
"""
from __future__ import print_function

import math
import copy
import threading
import numpy as np

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_from_euler, quaternion_from_matrix


class UnityCmdOdomBridge(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.cmd_topic = rospy.get_param("~cmd_topic", "/planning/pos_cmd")
        self.unity_odom_topics = rospy.get_param(
            "~unity_odom_topics",
            ["/drone_0_visual_slam/odom", "/odom"])
        if isinstance(self.unity_odom_topics, str):
            self.unity_odom_topics = [
                t.strip() for t in self.unity_odom_topics.split(",") if t.strip()]
        self.planner_odom_topic = rospy.get_param(
            "~planner_odom_topic", "/lidar_slam/odom")
        self.unity_feedback_odom_topic = rospy.get_param(
            "~unity_feedback_odom_topic", "/unity_odom")
        feedback = rospy.resolve_name(self.unity_feedback_odom_topic)
        outputs = self.unity_odom_topics + [self.planner_odom_topic]
        if feedback in [rospy.resolve_name(topic) for topic in outputs]:
            raise ValueError("Unity feedback topic must differ from bridge outputs")
        self.unity_cloud_topics = rospy.get_param(
            "~unity_cloud_topics",
            ["/drone_0_pcl_render_node/cloud", "/mid360/points"])
        if isinstance(self.unity_cloud_topics, str):
            self.unity_cloud_topics = [
                t.strip() for t in self.unity_cloud_topics.split(",") if t.strip()]
        self.planner_cloud_topic = rospy.get_param(
            "~planner_cloud_topic", "/cloud_registered")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "body")
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.init_x = float(rospy.get_param("~init_x", 0.0))
        self.init_y = float(rospy.get_param("~init_y", 0.0))
        self.init_z = float(rospy.get_param("~init_z", 1.5))
        self.init_yaw = float(rospy.get_param("~init_yaw", 0.0))

        self.pose = Pose()
        self.pose.position.x = self.init_x
        self.pose.position.y = self.init_y
        self.pose.position.z = self.init_z
        self._set_yaw(self.init_yaw)
        self.twist = Twist()
        self.have_cmd = False
        self.cloud_count = 0
        # Unity feedback may publish pose with an all-zero twist. The planner
        # needs measured motion, never the outgoing command's velocity.
        self.estimate_feedback_twist = bool(rospy.get_param("~estimate_feedback_twist", False))
        self.feedback_twist_tau = max(0.0, float(rospy.get_param("~feedback_twist_tau", 0.15)))
        self._feedback_previous = None
        self._feedback_velocity = [0.0, 0.0, 0.0]
        self._feedback_yaw_rate = 0.0

        self.unity_odom_pubs = [
            rospy.Publisher(topic, Odometry, queue_size=20)
            for topic in self.unity_odom_topics
        ]
        self.planner_odom_pub = rospy.Publisher(
            self.planner_odom_topic, Odometry, queue_size=20)
        self.cloud_pub = rospy.Publisher(
            self.planner_cloud_topic, PointCloud2, queue_size=1)

        rospy.Subscriber(self.cmd_topic, PositionCommand, self._on_cmd, queue_size=50)
        self.feedback_sub = rospy.Subscriber(
            self.unity_feedback_odom_topic, Odometry, self._on_feedback_odom,
            queue_size=1, tcp_nodelay=True)
        for topic in self.unity_cloud_topics:
            rospy.Subscriber(topic, PointCloud2, self._on_cloud, queue_size=1)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self._on_timer)
        rospy.loginfo(
            "[unity_bridge] hover=(%.2f, %.2f, %.2f) cmd=%s unity_odom=%s "
            "planner_odom=%s cloud %s -> %s",
            self.init_x, self.init_y, self.init_z,
            self.cmd_topic, ",".join(self.unity_odom_topics),
            self.planner_odom_topic,
            ",".join(self.unity_cloud_topics), self.planner_cloud_topic)
        rospy.loginfo("[unity_bridge] actual feedback %s -> %s (ROS reception stamps)",
                      self.unity_feedback_odom_topic, self.planner_odom_topic)

    def _set_yaw(self, yaw):
        q = quaternion_from_euler(0.0, 0.0, yaw)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

    def _on_cmd(self, msg):
        with self.lock:
            # Reconstruct the same drag-free flatness frame used by Gate. This
            # also supports HOLD/exploration messages without attitude fields.
            ax, ay, az = msg.acceleration.x, msg.acceleration.y, msg.acceleration.z + 9.81
            values = [msg.position.x,msg.position.y,msg.position.z,msg.velocity.x,msg.velocity.y,msg.velocity.z,
                      msg.jerk.x,msg.jerk.y,msg.jerk.z,msg.yaw_dot]
            if not all(math.isfinite(v) for v in values):
                return
            norm = math.sqrt(ax*ax + ay*ay + az*az)
            if not all(math.isfinite(v) for v in (ax, ay, az, msg.yaw)) or norm < 1e-6:
                rospy.logwarn_throttle(1.0, "[unity_bridge] invalid command flatness frame")
                return
            zb = np.array([ax, ay, az]) / norm
            yc = np.cross(zb, [math.cos(msg.yaw), math.sin(msg.yaw), 0.0])
            yn = np.linalg.norm(yc)
            if yn < 1e-6:
                return
            yb = yc / yn
            mat = np.eye(4)
            mat[:3, :3] = np.column_stack((np.cross(yb, zb), yb, zb))
            q = quaternion_from_matrix(mat)
            self.pose.position.x = msg.position.x
            self.pose.position.y = msg.position.y
            self.pose.position.z = msg.position.z
            self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = q
            jerk = np.array([msg.jerk.x, msg.jerk.y, msg.jerk.z])
            zbd = (jerk-zb*np.dot(zb,jerk))/norm
            xcd = msg.yaw_dot*np.array([-math.sin(msg.yaw), math.cos(msg.yaw), 0.0])
            yd = np.cross(zbd,[math.cos(msg.yaw),math.sin(msg.yaw),0.0])+np.cross(zb,xcd)
            ybd = (yd-yb*np.dot(yb,yd))/yn
            xbd = np.cross(ybd,zb)+np.cross(yb,zbd)
            skew = mat[:3,:3].T.dot(np.column_stack((xbd,ybd,zbd)))
            self.twist.angular.x = .5*(skew[2,1]-skew[1,2])
            self.twist.angular.y = .5*(skew[0,2]-skew[2,0])
            self.twist.angular.z = .5*(skew[1,0]-skew[0,1])
            self.twist.linear.x = msg.velocity.x
            self.twist.linear.y = msg.velocity.y
            self.twist.linear.z = msg.velocity.z
            self.have_cmd = True

    def _on_feedback_odom(self, msg):
        stamp = rospy.Time.now()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  v.x, v.y, v.z, w.x, w.y, w.z]
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(1.0, "[unity_bridge] discard nonfinite feedback")
            return
        if sum(value * value for value in [q.x, q.y, q.z, q.w]) < 1e-12:
            rospy.logwarn_throttle(1.0, "[unity_bridge] discard invalid orientation")
            return
        out = copy.deepcopy(msg)
        out.header.stamp = stamp
        if self.estimate_feedback_twist:
            sample_time = msg.header.stamp.to_sec() if not msg.header.stamp.is_zero() else stamp.to_sec()
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            current = (sample_time, (p.x, p.y, p.z), yaw)
            previous = self._feedback_previous
            self._feedback_previous = current
            dt = sample_time - previous[0] if previous is not None else 0.0
            if previous is not None and 1e-4 < dt <= 0.5:
                alpha = 1.0 - math.exp(-dt / self.feedback_twist_tau) if self.feedback_twist_tau > 0.0 else 1.0
                measured = [(current[1][i] - previous[1][i]) / dt for i in range(3)]
                delta_yaw = math.atan2(math.sin(yaw - previous[2]), math.cos(yaw - previous[2]))
                self._feedback_velocity = [old + alpha * (new - old)
                                           for old, new in zip(self._feedback_velocity, measured)]
                self._feedback_yaw_rate += alpha * (delta_yaw / dt - self._feedback_yaw_rate)
            else:
                self._feedback_velocity = [0.0, 0.0, 0.0]
                self._feedback_yaw_rate = 0.0
            out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = self._feedback_velocity
            out.twist.twist.angular.z = self._feedback_yaw_rate
        # Preserve supplied frames; changing a frame name is not a transform.
        out.header.frame_id = msg.header.frame_id or self.frame_id
        out.child_frame_id = msg.child_frame_id or self.child_frame_id
        self.planner_odom_pub.publish(out)

    def _on_cloud(self, msg):
        out = PointCloud2()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.frame_id if not msg.header.frame_id else msg.header.frame_id
        if not out.header.frame_id:
            out.header.frame_id = self.frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self.cloud_pub.publish(out)
        self.cloud_count += 1
        if self.cloud_count == 1 or self.cloud_count % 50 == 0:
            rospy.loginfo(
                "[unity_bridge] restamped Unity cloud #%d  %dx%d  frame=%s",
                self.cloud_count, out.width, out.height, out.header.frame_id)

    def _make_odom(self, stamp, pose, twist):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = pose
        odom.twist.twist = twist
        return odom

    def _on_timer(self, _event):
        stamp = rospy.Time.now()
        with self.lock:
            pose = copy.deepcopy(self.pose)
            twist = copy.deepcopy(self.twist)
        odom = self._make_odom(stamp, pose, twist)
        for pub in self.unity_odom_pubs:
            pub.publish(odom)


def main():
    rospy.init_node("unity_cmd_odom_bridge")
    UnityCmdOdomBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
