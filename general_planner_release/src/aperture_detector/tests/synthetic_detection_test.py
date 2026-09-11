#!/usr/bin/env python3
"""Run against a caller-owned, isolated ROS master; never starts a controller."""
import subprocess
import time

import rospy
from aperture_detector.msg import ApertureObservation
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Bool, String


def main():
    rospy.init_node('synthetic_aperture_test', anonymous=True)
    ns = '/polygon_hole_step_viz'
    params = dict(frame_id='world', lidar_topic='/test/aperture_cloud',
                  dynamic_roi_enable=False, dynamic_roi_odom_topic='/test/aperture_odom',
                  visual_roi_enable=False, transform_input_to_target_frame=False,
                  x_min=1., x_max=3., y_min=-1.2, y_max=1.2, z_min=-1.2, z_max=1.2,
                  accumulate_frames=3, min_frames_to_process=2, publish_rate=10.,
                  plane_polygon_use_largest_cluster=False, plane_polygon_use_concave_hull=False,
                  plane_polygon_tight_fit_enable=False, plane_boundary_max_unsupported_ratio=1.,
                  voxel_leaf=.01, hole_grid_resolution=.02, hole_occupancy_dilate_cells=1,
                  hole_boundary_support_radius=.08, hole_boundary_min_support=1,
                  hole_boundary_max_unsupported_ratio=.5, pass_area_ratio=.05,
                  publish_planning_snapshot_json=True, publish_overlay_image=False,
                  save_hole_polygon_txt=False, cloud_timeout=.7)
    for k, v in params.items():
        rospy.set_param(ns + '/' + k, v)
    seen, snapshots = [], []
    rospy.Subscriber(ns + '/observation', ApertureObservation, seen.append)
    rospy.Subscriber(ns + '/planning_snapshot_json', String, snapshots.append)
    cloud_pub = rospy.Publisher('/test/aperture_cloud', PointCloud2, queue_size=1)
    odom_pub = rospy.Publisher('/test/aperture_odom', Odometry, queue_size=1)
    enable_pub = rospy.Publisher(ns + '/detection_enable', Bool, queue_size=1)
    log = open('/tmp/aperture_synthetic_node.log', 'w')
    proc = subprocess.Popen(['rosrun', 'aperture_detector', 'polygon_hole_step_viz'], stdout=log, stderr=log)
    points = [(2., i*.02, j*.02) for i in range(-50, 51) for j in range(-50, 51)
              if abs(i) >= 22 or abs(j) >= 22]

    def feed(cloud_points, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            stamp = rospy.Time.now()
            odom = Odometry(header=Header(stamp=stamp, frame_id='world'))
            odom.pose.pose.orientation.w = 1.
            odom_pub.publish(odom)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='world'), cloud_points))
            time.sleep(.1)

    try:
        end = time.monotonic() + 8
        while cloud_pub.get_num_connections() == 0 and time.monotonic() < end:
            time.sleep(.1)
        assert cloud_pub.get_num_connections(), 'detector did not start'
        feed(points, 3.)
        valid = [x for x in seen if x.geometry_valid]
        assert valid, 'no valid rectangular aperture; inspect /tmp/aperture_synthetic_node.log'
        obs = valid[-1]
        assert abs(obs.center.x - 2.) < .03 and abs(obs.center.y) < .06 and abs(obs.center.z) < .06
        assert abs(obs.normal.x) > .99 and len(obs.boundary.points) >= 4
        assert not obs.depth_known and not obs.header.stamp.is_zero()
        assert snapshots, 'PASS did not produce compatibility JSON'
        time.sleep(1.2)
        assert not seen[-1].geometry_valid, 'stale cloud retained valid observation'
        enable_pub.publish(False)
        time.sleep(.3)
        assert not seen[-1].geometry_valid
        enable_pub.publish(True)
        feed([(2., i*.02, j*.02) for i in range(-50, 51) for j in range(-50, 51)], 2.)
        assert not seen[-1].geometry_valid, 'solid wall detected as aperture'
        # A detected hole below the configured PASS ratio must not emit a snapshot.
        proc.terminate()
        proc.wait(timeout=5)
        rospy.set_param(ns + '/pass_area_ratio', .95)
        seen.clear()
        snapshots.clear()
        proc = subprocess.Popen(['rosrun', 'aperture_detector', 'polygon_hole_step_viz'], stdout=log, stderr=log)
        time.sleep(1.)
        feed(points, 2.)
        assert seen and not seen[-1].geometry_valid, 'area-ratio FAIL accepted'
        assert not snapshots, 'area-ratio FAIL published a planning JSON'
        print('PASS: aperture geometry, JSON, observation timestamp, stale input, enable switch, solid wall, FAIL gating')
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()


if __name__ == '__main__':
    main()
