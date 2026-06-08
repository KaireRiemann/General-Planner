#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Point
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


POINT_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("intensity", 12, PointField.FLOAT32, 1),
]


DEFAULT_BOXES = [
    {
        "name": "crossing_near",
        "center": [0.0, -42.0, 1.6],
        "size": [1.0, 1.0, 1.5],
        "amplitude": [2.0, 0.0, 0.0],
        "period": 12.0,
        "phase": 0.0,
        "color": [1.0, 0.18, 0.08, 0.68],
    },
    {
        "name": "crossing_mid",
        "center": [2.0, -32.0, 1.7],
        "size": [1.2, 0.8, 1.6],
        "amplitude": [-2.2, 0.0, 0.0],
        "period": 15.0,
        "phase": 1.57,
        "color": [0.05, 0.55, 1.0, 0.68],
    },
    {
        "name": "drifting_far",
        "center": [-1.0, -22.0, 1.8],
        "size": [0.9, 1.3, 1.7],
        "amplitude": [1.4, 1.0, 0.0],
        "period": 18.0,
        "phase": 3.14,
        "color": [0.1, 0.85, 0.25, 0.68],
    },
    {
        "name": "gate_low",
        "center": [-2.5, -46.0, 1.4],
        "size": [0.8, 0.8, 1.2],
        "amplitude": [0.0, 1.8, 0.0],
        "period": 16.0,
        "phase": 0.6,
        "color": [1.0, 0.72, 0.05, 0.68],
    },
    {
        "name": "gate_high",
        "center": [2.8, -38.0, 2.0],
        "size": [0.9, 0.9, 1.3],
        "amplitude": [-1.6, 0.8, 0.0],
        "period": 19.0,
        "phase": 2.1,
        "color": [0.75, 0.2, 1.0, 0.68],
    },
    {
        "name": "side_sweeper_left",
        "center": [-3.0, -28.0, 1.6],
        "size": [1.1, 0.7, 1.4],
        "amplitude": [1.2, 0.0, 0.25],
        "period": 14.0,
        "phase": 1.2,
        "color": [0.0, 0.85, 0.85, 0.68],
    },
    {
        "name": "side_sweeper_right",
        "center": [3.0, -18.0, 1.7],
        "size": [1.1, 0.7, 1.4],
        "amplitude": [-1.2, 0.5, 0.0],
        "period": 17.0,
        "phase": 2.8,
        "color": [0.95, 0.35, 0.55, 0.68],
    },
    {
        "name": "slow_crossing",
        "center": [0.0, -12.0, 1.6],
        "size": [1.3, 0.9, 1.5],
        "amplitude": [2.5, 0.0, 0.0],
        "period": 24.0,
        "phase": 0.4,
        "color": [0.45, 0.9, 0.15, 0.68],
    },
    {
        "name": "diagonal_mid",
        "center": [-1.5, -36.0, 2.1],
        "size": [0.8, 1.2, 1.0],
        "amplitude": [1.0, 1.2, 0.0],
        "period": 21.0,
        "phase": 1.9,
        "color": [0.2, 0.35, 1.0, 0.68],
    },
    {
        "name": "hover_pulse",
        "center": [1.5, -25.0, 1.3],
        "size": [0.9, 0.9, 1.1],
        "amplitude": [0.8, -0.8, 0.2],
        "period": 13.0,
        "phase": 2.5,
        "color": [1.0, 0.45, 0.0, 0.68],
    },
    {
        "name": "rear_crossing_early",
        "center": [-0.8, 4.0, 1.6],
        "size": [1.1, 0.9, 1.5],
        "amplitude": [2.0, 0.0, 0.0],
        "period": 20.0,
        "phase": 0.9,
        "color": [0.1, 0.65, 1.0, 0.68],
    },
    {
        "name": "rear_gate_low",
        "center": [2.4, 10.0, 1.4],
        "size": [0.9, 0.8, 1.2],
        "amplitude": [0.0, 1.6, 0.0],
        "period": 18.0,
        "phase": 1.4,
        "color": [1.0, 0.6, 0.1, 0.68],
    },
    {
        "name": "rear_diagonal_sweeper",
        "center": [-2.2, 16.0, 1.9],
        "size": [0.9, 1.2, 1.4],
        "amplitude": [1.6, -1.0, 0.0],
        "period": 24.0,
        "phase": 2.4,
        "color": [0.55, 0.25, 1.0, 0.68],
    },
    {
        "name": "rear_side_sweeper_left",
        "center": [-3.0, 23.0, 1.6],
        "size": [1.0, 0.7, 1.4],
        "amplitude": [1.4, 0.0, 0.2],
        "period": 17.0,
        "phase": 0.3,
        "color": [0.0, 0.9, 0.7, 0.68],
    },
    {
        "name": "rear_side_sweeper_right",
        "center": [3.0, 29.0, 1.7],
        "size": [1.0, 0.8, 1.4],
        "amplitude": [-1.5, 0.6, 0.0],
        "period": 21.0,
        "phase": 2.0,
        "color": [0.95, 0.3, 0.55, 0.68],
    },
    {
        "name": "rear_slow_crossing",
        "center": [0.4, 35.0, 1.6],
        "size": [1.3, 0.9, 1.5],
        "amplitude": [2.4, 0.0, 0.0],
        "period": 28.0,
        "phase": 1.1,
        "color": [0.45, 0.9, 0.15, 0.68],
    },
    {
        "name": "rear_gate_high",
        "center": [-2.6, 41.0, 2.0],
        "size": [0.9, 0.9, 1.3],
        "amplitude": [1.5, -0.8, 0.0],
        "period": 23.0,
        "phase": 2.7,
        "color": [0.75, 0.2, 1.0, 0.68],
    },
    {
        "name": "rear_final_crossing",
        "center": [1.2, 46.0, 1.5],
        "size": [1.0, 1.0, 1.3],
        "amplitude": [-1.8, 0.0, 0.15],
        "period": 26.0,
        "phase": 0.5,
        "color": [1.0, 0.2, 0.15, 0.68],
    },
]


def vector_value(raw, default, length):
    if raw is None:
        return list(default)
    if isinstance(raw, (list, tuple)) and len(raw) >= length:
        return [float(raw[i]) for i in range(length)]
    return list(default)


def box_value(box, key, default):
    return box[key] if isinstance(box, dict) and key in box else default


class MovingObstacleMarkers:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.marker_topic = rospy.get_param("~marker_topic", "/perfect_drone/dynamic_obstacle_markers")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/perfect_drone/dynamic_obstacle_cloud")
        self.publish_cloud = bool(rospy.get_param("~publish_cloud", False))
        self.rate = max(1.0, float(rospy.get_param("~rate", 15.0)))
        self.start_after = max(0.0, float(rospy.get_param("~start_after", 0.0)))
        self.cloud_resolution = max(0.05, float(rospy.get_param("~cloud_resolution", 0.2)))
        self.cloud_intensity = float(rospy.get_param("~cloud_intensity", 20.0))
        self.max_cloud_points = max(100, int(rospy.get_param("~max_cloud_points", 30000)))
        self.boxes = rospy.get_param("~boxes", DEFAULT_BOXES)
        if not isinstance(self.boxes, list):
            rospy.logwarn("~boxes must be a list. Falling back to default moving boxes.")
            self.boxes = DEFAULT_BOXES

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=2)
        self.cloud_pub = None
        if self.publish_cloud:
            self.cloud_pub = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=2)
        self.start_time = rospy.Time.now()

    def center_at(self, box, elapsed):
        center = vector_value(box_value(box, "center", [0.0, 0.0, 1.6]), [0.0, 0.0, 1.6], 3)
        amplitude = vector_value(box_value(box, "amplitude", [0.0, 0.0, 0.0]), [0.0, 0.0, 0.0], 3)
        period = max(0.1, float(box_value(box, "period", 10.0)))
        phase = float(box_value(box, "phase", 0.0))
        angle = 2.0 * math.pi * elapsed / period + phase
        return [
            center[0] + amplitude[0] * math.sin(angle),
            center[1] + amplitude[1] * math.cos(angle),
            center[2] + amplitude[2] * math.sin(angle),
        ]

    def make_marker(self, stamp, box, marker_id, elapsed, center=None):
        if center is None:
            center = self.center_at(box, elapsed)
        size = vector_value(box_value(box, "size", [1.0, 1.0, 1.5]), [1.0, 1.0, 1.5], 3)
        color = vector_value(box_value(box, "color", [1.0, 0.18, 0.08, 0.68]),
                             [1.0, 0.18, 0.08, 0.68],
                             4)

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = str(box_value(box, "name", "dynamic_obstacle"))
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = Point(*center)
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.02, size[0])
        marker.scale.y = max(0.02, size[1])
        marker.scale.z = max(0.02, size[2])
        marker.color.r = min(max(color[0], 0.0), 1.0)
        marker.color.g = min(max(color[1], 0.0), 1.0)
        marker.color.b = min(max(color[2], 0.0), 1.0)
        marker.color.a = min(max(color[3], 0.0), 1.0)
        marker.lifetime = rospy.Duration(2.0 / self.rate)
        return marker

    def sample_box_surface(self, box, center):
        size = vector_value(box_value(box, "size", [1.0, 1.0, 1.5]), [1.0, 1.0, 1.5], 3)
        half = [0.5 * max(0.02, value) for value in size]
        intensity = float(box_value(box, "intensity", self.cloud_intensity))
        axes = []
        for axis in range(3):
            count = max(2, int(math.ceil(size[axis] / self.cloud_resolution)) + 1)
            if count == 1:
                axes.append([center[axis]])
                continue
            start = center[axis] - half[axis]
            step = size[axis] / float(count - 1)
            axes.append([start + step * idx for idx in range(count)])

        points = []
        for fixed_axis in range(3):
            other_axes = [axis for axis in range(3) if axis != fixed_axis]
            for fixed_value in (center[fixed_axis] - half[fixed_axis],
                                center[fixed_axis] + half[fixed_axis]):
                for v0 in axes[other_axes[0]]:
                    for v1 in axes[other_axes[1]]:
                        point = [0.0, 0.0, 0.0]
                        point[fixed_axis] = fixed_value
                        point[other_axes[0]] = v0
                        point[other_axes[1]] = v1
                        points.append((point[0], point[1], point[2], intensity))
        return points

    def publish_cloud_msg(self, stamp, points):
        if self.cloud_pub is None:
            return
        if len(points) > self.max_cloud_points:
            rospy.logwarn_throttle(1.0,
                                   "Dynamic obstacle cloud has %d points, truncating to %d",
                                   len(points),
                                   self.max_cloud_points)
            points = points[:self.max_cloud_points]
        header = Header(stamp=stamp, frame_id=self.frame_id)
        self.cloud_pub.publish(point_cloud2.create_cloud(header, POINT_FIELDS, points))

    def spin(self):
        rate = rospy.Rate(self.rate)
        rospy.loginfo("Publishing %d moving obstacle boxes on %s", len(self.boxes), self.marker_topic)
        if self.publish_cloud:
            rospy.loginfo("Publishing moving obstacle point cloud on %s", self.cloud_topic)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            elapsed = (now - self.start_time).to_sec()
            marker_array = MarkerArray()
            cloud_points = []
            if elapsed >= self.start_after:
                active_elapsed = elapsed - self.start_after
                for idx, box in enumerate(self.boxes):
                    center = self.center_at(box, active_elapsed)
                    marker_array.markers.append(self.make_marker(now, box, idx, active_elapsed, center))
                    if self.publish_cloud:
                        cloud_points.extend(self.sample_box_surface(box, center))
            self.marker_pub.publish(marker_array)
            if self.publish_cloud:
                self.publish_cloud_msg(now, cloud_points)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("moving_obstacle_markers")
    MovingObstacleMarkers().spin()
