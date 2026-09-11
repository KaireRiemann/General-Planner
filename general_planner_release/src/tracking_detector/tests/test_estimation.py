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

if __name__=="__main__":unittest.main()
