#!/usr/bin/env python3
"""Verify turn propagation and epoch alignment on an isolated ROS master."""
import json
import math
import os
import signal
import subprocess
import time

os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:11329"
import rosgraph
try:
    rosgraph.Master("/probe").getPid()
except Exception:
    pass
else:
    raise RuntimeError("isolated port 11329 is already in use")
processes = []


def spawn(command):
    with open("/tmp/target_prediction_test_%d.log" % len(processes),"w") as log:
        process = subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True)
    processes.append(process)


try:
    spawn(["roscore","-p","11329"])
    for _ in range(60):
        try:
            rosgraph.Master("/probe").getPid()
            break
        except Exception:
            time.sleep(.1)
    import rospy
    from nav_msgs.msg import Odometry,Path
    from std_msgs.msg import Bool,Float64
    rospy.init_node("prediction_regression")
    spawn(["rosrun","tracking_detector","target_path_predictor_node",
           "__name:=predict","_require_target_valid:=true","_prediction_horizon:=2.5",
           "_minimum_prediction_horizon:=1.0","_enable_turn_prediction:=true","_input_is_cv_extrapolated:=true",
           "~target_odom:=/target","~target_valid:=/valid",
           "~observation_age:=/age","~prediction:=/prediction"])
    odom=rospy.Publisher("/target",Odometry,queue_size=1)
    valid=rospy.Publisher("/valid",Bool,queue_size=1,latch=True)
    age=rospy.Publisher("/age",Float64,queue_size=1,latch=True)
    paths=[]
    rospy.Subscriber("/prediction",Path,lambda m:paths.append(m))
    results={}
    for scenario,rate in (("straight",0.),("turn",.8),("delayed_turn",.8),("noisy_straight",0.),("noisy_delayed_turn",.8)):
        valid.publish(False)
        time.sleep(.3)
        paths.clear()
        epoch=rospy.Time.now().to_sec()
        start=time.monotonic()
        while time.monotonic()-start < 6.:
            valid.publish(True)
            delay=.5 if scenario in ("delayed_turn","noisy_delayed_turn") else 0.
            age.publish(delay if scenario!="noisy_straight" else .4)
            # Input state is deliberately 80 ms older than the output path epoch.
            stamp=rospy.Time.now()-rospy.Duration(.08)
            t=stamp.to_sec()-epoch-delay
            m=Odometry();m.header.stamp=stamp;m.header.frame_id="world"
            m.pose.pose.position.x=2./rate*math.sin(rate*t) if rate else 2.*t
            m.pose.pose.position.y=2./rate*(1.-math.cos(rate*t)) if rate else 0.
            m.pose.pose.position.z=.7
            m.pose.pose.orientation.w=1.
            m.twist.twist.linear.x=2.*math.cos(rate*t)
            m.twist.twist.linear.y=2.*math.sin(rate*t)
            m.pose.pose.position.x+=delay*m.twist.twist.linear.x
            m.pose.pose.position.y+=delay*m.twist.twist.linear.y
            m.pose.covariance[0]=m.pose.covariance[7]=m.pose.covariance[14]=.01
            if scenario.startswith("noisy_"):
                m.twist.twist.linear.y+=.65*math.sin(5.*t)
                m.pose.pose.position.y+=.12*math.sin(3.*t)
                m.pose.covariance[0]=m.pose.covariance[7]=.2
                m.twist.covariance[0]=m.twist.covariance[7]=.35
            odom.publish(m)
            time.sleep(.05)
        recent=[m for m in paths if m.poses and m.header.stamp.to_sec()-epoch>4.]
        assert len(recent)>10, "no sustained predictions"
        errors=[]
        for path in recent:
            assert len(path.poses)>=6, "age shrink collapsed the prediction horizon"
            for pose in path.poses:
                t=pose.header.stamp.to_sec()-epoch
                x=2./rate*math.sin(rate*t) if rate else 2.*t
                y=2./rate*(1.-math.cos(rate*t)) if rate else 0.
                errors.append(math.hypot(pose.pose.position.x-x,pose.pose.position.y-y))
        if scenario=="noisy_straight":
            curvatures=[]
            for path in recent:
                first,last=path.poses[0],path.poses[-1]
                yaw=lambda q:2.*math.atan2(q.z,q.w)
                curvatures.append(abs(math.remainder(yaw(last.pose.orientation)-yaw(first.pose.orientation),2.*math.pi))/
                                  (last.header.stamp-first.header.stamp).to_sec())
            assert max(curvatures)<.02, ("velocity jitter fabricated turns",max(curvatures))
            assert max(errors)<.5, ("uncertain lateral velocity was not stabilized",max(errors))
        elif scenario=="noisy_delayed_turn":
            assert max(errors)<1.0, ("noisy delayed turn model error",max(errors))
            # True sustained rotation must survive covariance-aware fitting.
            rates=[]
            for path in recent:
                first,last=path.poses[0],path.poses[-1]
                yaw=lambda q:2.*math.atan2(q.z,q.w)
                rates.append(math.remainder(yaw(last.pose.orientation)-yaw(first.pose.orientation),2.*math.pi)/
                             (last.header.stamp-first.header.stamp).to_sec())
            assert min(rates)>.45 and max(rates)<1.1, ("true turn lost or exaggerated",min(rates),max(rates))
        else:
            assert max(errors)<.08, ("prediction or publication epoch mismatch",rate,max(errors))
        valid.publish(False)
        time.sleep(.3)
        assert paths and not paths[-1].poses, "invalidation did not retire the path"
        results[scenario]=dict(maximum_prediction_error=max(errors),
            minimum_path_samples=min(len(m.poses) for m in recent),invalidation_passed=True)
        if scenario=="noisy_straight": results[scenario]["maximum_false_turn_rate"]=max(curvatures)
        if scenario=="noisy_delayed_turn": results[scenario]["turn_rate_range"]=[min(rates),max(rates)]
    print(json.dumps(results),flush=True)

finally:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
