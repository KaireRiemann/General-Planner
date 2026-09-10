#!/usr/bin/env python3
"""Real CPU inference -> bbox -> EKF -> Path smoke test on an isolated master.
Run launch with target_label:=car and the default topic arguments.
"""
import math
import os
import time
from pathlib import Path
import cv2
import numpy as np
import rospkg
import rospy
from nav_msgs.msg import Odometry, Path as Prediction
from sensor_msgs.msg import CompressedImage
from tracking_detector.msg import BoundingBoxes

if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11329':
    raise RuntimeError('This test publishes synthetic sensor data; use isolated master port 11329')
rospy.init_node('tracking_detector_smoke')
root = Path(rospkg.RosPack().get_path('tracking_detector'))
photo = cv2.imread(str(root / 'tests/data/car.jpg'))
assert photo is not None
counts = {'car': 0, 'empty': 0, 'odom': 0, 'path': 0}

def boxes(m):
    assert m.header.stamp == m.image_header.stamp
    if not m.bounding_boxes:
        counts['empty'] += 1
    for b in m.bounding_boxes:
        assert 0 <= b.xmin < b.xmax <= 640 and 0 <= b.ymin < b.ymax <= 480
        if b.Class == 'car':
            counts['car'] += 1

def target(m):
    p = m.pose.pose.position
    assert all(math.isfinite(v) for v in (p.x, p.y, p.z))
    assert m.header.frame_id == 'world'
    counts['odom'] += 1

def path(m):
    assert len(m.poses) == 17
    assert abs((m.poses[-1].header.stamp - m.poses[0].header.stamp).to_sec() - 4) < 1e-6
    counts['path'] += 1

subs = [rospy.Subscriber('/tracking/bboxes', BoundingBoxes, boxes),
        rospy.Subscriber('/target_ekf_node/target_odom', Odometry, target),
        rospy.Subscriber('/tracking/target_prediction', Prediction, path)]
rgb = rospy.Publisher('/camera0/color/image/compressed', CompressedImage, queue_size=1)
odom = rospy.Publisher('/unity_odom', Odometry, queue_size=20)
def send(im):
    now = rospy.Time.now()
    o = Odometry(); o.header.stamp = now; o.header.frame_id = 'world'
    o.pose.pose.orientation.w = 1; o.pose.pose.position.z = 1.5
    m = CompressedImage(); m.header.stamp = now; m.header.frame_id = 'camera'
    m.format = 'jpeg'; m.data = cv2.imencode('.jpg', im)[1].tobytes()
    odom.publish(o); rgb.publish(m)
end = time.monotonic() + 60
while time.monotonic() < end and not all(counts[k] for k in ('car', 'odom', 'path')):
    send(photo); time.sleep(.1)
assert all(counts[k] for k in ('car', 'odom', 'path')), counts
end = time.monotonic() + 15
while time.monotonic() < end and not counts['empty']:
    send(np.zeros((480, 640, 3), dtype=np.uint8)); time.sleep(.1)
assert counts['empty'], counts
print('PASS real YOLOE -> bbox -> EKF -> prediction, including empty detections:', counts)
