#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import threading
import json
from collections import deque
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from tf.transformations import quaternion_from_euler
spec = importlib.util.spec_from_file_location("estimator", Path(__file__).resolve().parents[1]/"scripts/target_state_estimator.py")
e = importlib.util.module_from_spec(spec)
spec.loader.exec_module(e)

class EstimationTests(unittest.TestCase):
    def test_variable_dt_velocity_and_outlier(self):
        f=e.ConstantVelocityFilter()
        t=0.
        for i in range(100):
            t += [0.08,0.16,0.11][i%3]
            self.assertTrue(f.update([2*t,1,0],0.01,t))
        self.assertAlmostEqual(f.x[3],2,delta=.04)
        previous=f.x.copy()
        self.assertFalse(f.update([200,100,0],0.01,t+.1))
        np.testing.assert_equal(f.x,previous)
        self.assertFalse(f.update([1,1,0],0.01,t-.1))
        self.assertGreaterEqual(np.linalg.eigvalsh(f.P).min(),-1e-10)

    @patch.object(e.rospy.Time,"now",return_value=e.rospy.Time(1000))
    def test_weak_boxes_only_maintain_associated_tracks(self,_clock):
        from tracking_detector.msg import BoundingBoxes,BoundingBox
        with patch.object(e.rospy,"get_param",side_effect=lambda key,default=None:default), \
             patch.object(e.rospy,"Publisher"),patch.object(e.rospy,"Subscriber"), \
             patch.object(e.rospy,"Timer"),patch.object(e.rospy,"on_shutdown"):
            node=e.TargetEstimator()
        node.info=SimpleNamespace(K=[415.69,0,320,0,415.69,240,0,0,1],width=640,height=480,D=[])
        node.odom=deque([(100.,np.array([0.,0.,1.5]),np.array([0.,0.,0.,1.])),
                         (100.1,np.array([0.,0.,1.5]),np.array([0.,0.,0.,1.]))])
        node.sensor_now=lambda:100.1
        def message(stamp,xmin=285,xmax=355,confidence=.45):
            m=BoundingBoxes();m.header.stamp=e.rospy.Time.from_sec(stamp)
            b=BoundingBox();b.Class="car";b.probability=confidence
            b.xmin=xmin;b.xmax=xmax;b.ymin=205;b.ymax=275
            m.bounding_boxes=[b];return m
        node.process(message(100.01))
        self.assertIsNone(node.filter.x,"weak box initialized a target")
        node.filter.update([8.,0.,.7],.1,100.02)
        node.count=node.confirmations;node.was_valid=True
        node.process(message(100.04))
        self.assertEqual(node.filter.stamp,100.04,"associated weak observation was dropped")
        node.process(message(100.06,xmin=520,xmax=590))
        self.assertEqual(node.filter.stamp,100.04,"weak unrelated target bypassed association")
        node.was_valid=False;node.count=0
        node.process(message(100.08))
        self.assertEqual(node.count,0,"weak observation counted toward reacquisition")

    def test_ground_invariance_under_camera_motion(self):
        K=[415.6922,415.6922,320,240]
        Rcb=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        target=np.array([10.,2.,0.])
        for roll,pitch,yaw in [(0,0,-.2),(0,0,0),(0,0,.2),(0,0,.4),(.35,.4,.2),(-.4,-.25,-.2)]:
            p=np.array([1.,-.4,1.6])
            R=e.rotation(quaternion_from_euler(roll,pitch,yaw))@Rcb
            optical=R.T@(target-p)
            u=K[0]*optical[0]/optical[2]+K[2]
            v=K[1]*optical[1]/optical[2]+K[3]
            np.testing.assert_allclose(e.ground_point(u,v,K,p,R,0),target,atol=1e-8)
        with self.assertRaises(ValueError):
            e.ground_point(320,200,K,p,Rcb,0)

    def test_metric_depth_and_alignment_gate(self):
        estimator=object.__new__(e.TargetEstimator)
        estimator.info=SimpleNamespace(K=[415.69,0,320,0,415.69,240,0,0,1],width=640,height=480,D=[])
        estimator.width,estimator.height=640,480
        estimator.pcb=np.zeros(3)
        estimator.Rcb=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        estimator.range_method="depth"
        estimator.depth_registered=True
        estimator.depth_tolerance=.08
        estimator.max_range=30
        depth=np.full((480,640),20.)
        depth[220:261,300:341]=8.
        estimator.depth=[(100.,depth)]
        box=SimpleNamespace(xmin=290,xmax=350,ymin=210,ymax=270)
        p,var,method=estimator.measure(box,100.,np.zeros(3),np.eye(3))
        np.testing.assert_allclose(p,[8,0,0])
        self.assertEqual(method,"depth")
        estimator.depth_registered=False
        with self.assertRaises(ValueError):estimator.measure(box,100.,np.zeros(3),np.eye(3))
        estimator.depth_registered=True
        with self.assertRaises(ValueError):estimator.measure(box,101.,np.zeros(3),np.eye(3))

    def test_latency_gap_and_reconfirmation(self):
        node=object.__new__(e.TargetEstimator)
        node.lock=threading.RLock(); node.pending=deque()
        node.odom=deque([(100.,np.zeros(3),np.array([0.,0,0,1]))])
        node.filter=e.ConstantVelocityFilter()
        node.filter.update([8,0,.7],.01,99.5)
        node.last_odom_receipt=1000.; node.last_accept_wall=999.98
        node.max_age=1.; node.max_receipt_gap=.45; node.fresh_age=.25
        node.max_position_std=2.; node.confirmations=3; node.count=3
        node.loss_position_std=2.5; node.last_measurement_std=None
        node.pose_source="odometry_interpolation";node.height_scale=1.
        node.was_valid=False; node.track_id=1; node.reason="accepted"
        node.method="ground_plane"; node.output_frame="world"
        class Pub:
            def __init__(self):self.items=[]
            def publish(self,m):self.items.append(m)
        for n in ["valid_pub","age_pub","status_pub","odom_pub"]:
            setattr(node,n,Pub())
        with patch.object(e.rospy.Time,"now",return_value=e.rospy.Time(1000)), patch.object(e.time,"monotonic",return_value=1000.):
            node.tick(None)
        self.assertTrue(node.valid_pub.items[-1])
        self.assertEqual(json.loads(node.status_pub.items[-1])["state"],"tracking")
        # Recent odometry cannot hide a gap in accepted detections.
        node.last_odom_receipt=1000.5; node.odom[-1]=(100.5,np.zeros(3),np.array([0.,0,0,1]))
        with patch.object(e.rospy.Time,"now",return_value=e.rospy.Time(1001)), patch.object(e.time,"monotonic",return_value=1000.5):
            node.tick(None)
            self.assertFalse(node.valid_pub.items[-1])
            self.assertEqual(node.count,0)
            node.filter.update([8,0,.7],.01,100.4)
            node.last_accept_wall=1000.5; node.count=1
            node.tick(None)
            self.assertFalse(node.valid_pub.items[-1])
            node.count=3; node.tick(None)
            self.assertTrue(node.valid_pub.items[-1])

    def test_interpolation_and_clock_gap(self):
        samples=[(1.,np.zeros(3),np.array([0.,0,0,1])),
                 (1.2,np.array([2.,0,0]),np.array([0.,0,np.sin(.1),np.cos(.1)]))]
        p,R=e.interpolate_pose(samples,1.1,.15)
        np.testing.assert_allclose(p,[1,0,0],atol=1e-8)
        self.assertAlmostEqual(np.arctan2(R[1,0],R[0,0]),.1)
        with self.assertRaises(ValueError):e.interpolate_pose(samples,3.,.15)

    def test_anisotropic_measurement_covariance(self):
        f=e.ConstantVelocityFilter()
        R=np.diag([4.,.01,.01])
        self.assertTrue(f.update([12.,0.,.7],R,1.))
        np.testing.assert_allclose(f.P[:3,:3],R)
        self.assertTrue(f.update([12.2,0.,.7],R,1.1))
        self.assertFalse(f.update([12.4,10.,.7],R,1.2))
        self.assertFalse(f.update([12.4,0.,.7],np.diag([-1.,1.,1.]),1.2))
        self.assertGreater(np.linalg.eigvalsh(f.P).min(),0.)

    def test_known_height_compensates_capture_attitude(self):
        node=object.__new__(e.TargetEstimator)
        node.info=SimpleNamespace(K=[415.6922,0,320,0,415.6922,240,0,0,1],width=640,height=480,D=[])
        node.width,node.height=640,480;node.pcb=np.zeros(3)
        node.pose_source="capture_pose"
        node.Rcb=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        node.range_method="auto";node.depth_registered=False
        node.object_height=1.4;node.height_relative_std=.1;node.height_scale=1.
        node.ground_z=0.;node.center_height=.7
        p=np.array([0.,0.,1.5])
        for pitch in [-.15,0.,.15]:
            R=e.rotation(quaternion_from_euler(0.,pitch,0.))
            def project(z):
                q=(R@node.Rcb).T@(np.array([15.,0.,z])-p)
                return 240.+415.6922*q[1]/q[2]
            box=SimpleNamespace(xmin=300.,xmax=340.,ymin=project(1.4),ymax=project(0.))
            measured,covariance,method=node.measure(box,1.,p,R)
            self.assertEqual(method,"known_height")
            np.testing.assert_allclose(measured,[15.,0.,.7],atol=.03)
            self.assertGreater(np.linalg.eigvalsh(covariance).min(),0.)
        box.xmin=0
        with self.assertRaisesRegex(ValueError,"horizontal_bbox_clipped"):
            node.measure(box,1.,p,np.eye(3))

    def test_uncertainty_hysteresis_keeps_tracking_but_requires_reacquisition(self):
        node=object.__new__(e.TargetEstimator)
        node.lock=threading.RLock();node.pending=deque()
        node.odom=deque([(100.,np.zeros(3),np.array([0.,0,0,1]))])
        node.filter=e.ConstantVelocityFilter();node.filter.update([8.,0.,.7],.01,100.)
        node.last_odom_receipt=node.last_accept_wall=1000.
        node.max_age=1.;node.max_receipt_gap=.45;node.fresh_age=.25
        node.max_position_std=2.;node.loss_position_std=2.5
        node.confirmations=node.count=3;node.was_valid=True;node.track_id=1
        node.reason="accepted";node.method="known_height";node.output_frame="world"
        node.last_measurement_std=None;node.pose_source="odometry_interpolation";node.height_scale=1.
        class Pub:
            def __init__(self):self.items=[]
            def publish(self,m):self.items.append(m)
        for name in ["valid_pub","age_pub","status_pub","odom_pub"]:setattr(node,name,Pub())
        with patch.object(e.time,"monotonic",return_value=1000.),patch.object(e.rospy.Time,"now",return_value=e.rospy.Time(1000)):
            for sigma in [2.04,1.98,2.08,2.4]:
                node.filter.P[:3,:3]=np.eye(3)*sigma**2
                node.tick(None);self.assertTrue(node.valid_pub.items[-1])
            node.filter.P[:3,:3]=np.eye(3)*2.6**2
            node.tick(None);self.assertFalse(node.valid_pub.items[-1]);self.assertEqual(node.count,0)
            node.count=3;node.filter.P[:3,:3]=np.eye(3)*2.2**2
            node.tick(None);self.assertFalse(node.valid_pub.items[-1])
            node.filter.P[:3,:3]=np.eye(3)*1.9**2
            node.tick(None);self.assertTrue(node.valid_pub.items[-1])

if __name__=="__main__":unittest.main()
