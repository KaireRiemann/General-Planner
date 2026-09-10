#!/usr/bin/env python3
"""Synthetic LiDAR -> migrated detector -> real internal Gate -> ideal plant.
Run only against a caller-owned isolated master. No real vehicle or map fusion.
"""
import os
import subprocess
import time
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def main():
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11381':
        raise RuntimeError('requires isolated ROS_MASTER_URI=http://127.0.0.1:11381')
    rospy.init_node('internal_gate_detector_test')
    ns = '/polygon_hole_step_viz'
    params = dict(frame_id='world', lidar_topic='/gate_test/cloud',
                  dynamic_roi_enable=False, dynamic_roi_odom_topic='/gate_test/odom',
                  visual_roi_enable=False, transform_input_to_target_frame=False,
                  x_min=1., x_max=3., y_min=-1.2, y_max=1.2, z_min=0., z_max=2.4,
                  accumulate_frames=3, min_frames_to_process=2, publish_rate=10.,
                  plane_polygon_use_largest_cluster=False, plane_polygon_use_concave_hull=False,
                  plane_polygon_tight_fit_enable=False, plane_boundary_max_unsupported_ratio=1.,
                  voxel_leaf=.01, hole_grid_resolution=.02, hole_occupancy_dilate_cells=1,
                  hole_boundary_support_radius=.08, hole_boundary_min_support=1,
                  hole_boundary_max_unsupported_ratio=.5, pass_area_ratio=.05,
                  publish_planning_snapshot_json=False, publish_overlay_image=False,
                  save_hole_polygon_txt=False, cloud_timeout=.7)
    for key, value in params.items():
        rospy.set_param(ns + '/' + key, value)
    pub = rospy.Publisher('/gate_test/cloud', PointCloud2, queue_size=1)
    points = [(2., i*.02, 1.2+j*.02) for i in range(-50, 51) for j in range(-50, 51)
              if abs(i) >= 30 or abs(j) >= 30]
    detector = subprocess.Popen(['rosrun', 'aperture_detector', 'polygon_hole_step_viz'],
                                stdout=open('/tmp/internal_gate_detector.log', 'w'), stderr=subprocess.STDOUT)
    fixture = None
    try:
        deadline = time.monotonic()+10
        while pub.get_num_connections()==0 and time.monotonic()<deadline:
            time.sleep(.1)
        assert pub.get_num_connections(), 'detector failed to start'
        fixture = subprocess.Popen(['rosrun', 'general_planner', 'internal_gate_integration_test', 'detector'])
        deadline = time.monotonic()+55
        while fixture.poll() is None and time.monotonic()<deadline:
            pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=rospy.Time.now(), frame_id='world'), points))
            time.sleep(.1)
        assert fixture.poll()==0, 'detector-to-gate fixture failed or timed out'
        print('PASS: synthetic LiDAR -> aperture selection -> internal SE3 -> gateway -> measured completion')
    finally:
        for proc in [fixture, detector]:
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()


if __name__ == '__main__':
    main()
