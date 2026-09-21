#!/usr/bin/env python3
"""Closed command-feedback regression; owns its ROS master and synthetic inputs."""
import argparse
import bisect
import collections
import numpy as np
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path
import rospkg
import yaml

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--release", action="store_true")
parser.add_argument("--initial-yaw", type=float, default=0.0)
parser.add_argument("--initial-height", type=float, default=1.5)
parser.add_argument("--duration", type=float, default=22.0)
parser.add_argument("--scenario", choices=("uturn", "headon", "straight", "approach"), default="uturn")
parser.add_argument("--observation-delay", type=float, default=.2)
parser.add_argument("--feedback-delay", type=float, default=0.)
parser.add_argument("--sensor-fov", action="store_true")
parser.add_argument("--noisy-observation", action="store_true")
parser.add_argument("--prediction-node", action="store_true")
parser.add_argument("--obstacle", action="store_true")
args = parser.parse_args()
profile_package="general_planner_release" if args.release else "general_planner"
profile_path=Path(rospkg.RosPack().get_path(profile_package))/"config/task_planner_runtime_tracking.yaml"
tracking_profile=yaml.safe_load(profile_path.read_text())["general_planner"]["tracking"]
desired_distance=float(tracking_profile["distance"])
desired_height_offset=float(tracking_profile["height_offset"])
os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:11329"
import rosgraph

try:
    rosgraph.Master("/probe").getPid()
except Exception:
    pass
else:
    raise RuntimeError("isolated master port 11329 is already in use")


def car(t):
    if args.scenario in ("straight","approach"):
        return ((16. if args.scenario=="approach" else 8.)+2.*t,0.,2.,0.)
    if args.scenario == "headon":
        return (18.-2.*t, 0., -2., 0.)
    turn = max(0., min(math.pi/.8, t-7.))
    angle = .8*turn
    after = max(0., t-7.-math.pi/.8)
    return (8.+2.*min(t,7.)+2.5*math.sin(angle)-2.*after,
            2.5*(1.-math.cos(angle)), 2.*math.cos(angle), 2.*math.sin(angle))


processes = []


def spawn(command, name):
    with open("/tmp/tracking_"+args.scenario+"_"+name+".log", "w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
    processes.append(process)
    return process


try:
    spawn(["roscore", "-p", "11329"], "master")
    for _ in range(60):
        try:
            rosgraph.Master("/probe").getPid()
            break
        except Exception:
            time.sleep(.1)
    import rospy
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Header, String, Bool, Float64
    from quadrotor_msgs.msg import PositionCommand
    from general_planner.msg import PlannerStatus

    rospy.init_node("tracking_turn_safety_regression")
    package = "general_planner_release" if args.release else "task_planner"
    spawn(["roslaunch", package, "planner_runtime.launch", "initial_mode:=tracking",
           "tracking_detector:=false", "perceptor:=false", "rviz:=false",
           "auto_rviz_switch:=false", "enable_exploration:=false"], "runtime")
    predictor=None
    if args.prediction_node:
        predictor_command=([str(profile_path.parents[2]/"tracking_detector/target_path_predictor_node")]
            if args.release else ["rosrun","tracking_detector","target_path_predictor_node"])
        predictor=spawn(predictor_command+["__name:=test_predictor",
               "_require_target_valid:=true","_enable_turn_prediction:=true","_input_is_cv_extrapolated:=true","_prediction_horizon:=2.5",
               "_minimum_prediction_horizon:=1.0","~target_odom:=/test/target",
               "~target_valid:=/test/valid","~observation_age:=/test/age",
               "~prediction:=/tracking/target_prediction"],"predictor")
    history=collections.deque(maxlen=500)
    latest = [None]
    commands, events, phases, separations = [], [], [], []
    start = time.monotonic()

    def command_callback(m):
        latest[0] = m
        history.append((time.monotonic(),m))
        commands.append([time.monotonic()-start, m.position.x, m.position.y, m.position.z,
                         m.velocity.x, m.velocity.y, m.velocity.z,
                         m.acceleration.x, m.acceleration.y, m.acceleration.z,
                         m.jerk.x, m.jerk.y, m.jerk.z,m.yaw,m.yaw_dot])

    rospy.Subscriber("/planning/pos_cmd", PositionCommand, command_callback)
    rospy.Subscriber("/planning/diagnostics/events", String,
                     lambda m: events.append((time.monotonic()-start, m.data)))
    rospy.Subscriber("/planner/status", PlannerStatus,
                     lambda m: phases.append((time.monotonic()-start, m.phase_str, m.reason)))
    odom = rospy.Publisher("/lidar_slam/odom", Odometry, queue_size=1)
    cloud = rospy.Publisher("/cloud_registered", PointCloud2, queue_size=1)
    target = rospy.Publisher("/tracking/target_prediction", Path, queue_size=1)
    target_odom=rospy.Publisher("/test/target",Odometry,queue_size=1)
    target_valid=rospy.Publisher("/test/valid",Bool,queue_size=1,latch=True)
    observation_age=rospy.Publisher("/test/age",Float64,queue_size=1)
    visibility=[]
    last_seen=-1.e6
    floor = [[i*.4,j*.4,0.] for i in range(-80,100) for j in range(-35,36)]
    if args.obstacle:
        floor += [[5.+dx,1.5+dy,z] for dx in (-.3,0.,.3)
                  for dy in np.arange(-.6,.61,.15) for z in np.arange(.15,2.51,.15)]
    last_cloud = last_prediction = 0.
    while time.monotonic()-start < args.duration:
        assert predictor is None or predictor.poll() is None, "prediction node exited; inspect predictor log"
        now = time.monotonic()
        elapsed = max(0., now-start-3.)
        stamp = rospy.Time.now()
        m = Odometry()
        m.header = Header(stamp=stamp, frame_id="world")
        m.pose.pose.position.z = args.initial_height
        m.pose.pose.position.y = 2.5 if args.obstacle else 0.
        m.pose.pose.orientation.z = math.sin(args.initial_yaw/2.)
        m.pose.pose.orientation.w = math.cos(args.initial_yaw/2.)
        if latest[0]:
            samples=list(history)
            index=max(0,bisect.bisect_right([q[0] for q in samples],now-args.feedback_delay)-1)
            c=samples[index][1]
            m.pose.pose.position = c.position
            m.twist.twist.linear = c.velocity
            m.pose.pose.orientation.z = math.sin(c.yaw/2.)
            m.pose.pose.orientation.w = math.cos(c.yaw/2.)
        # Model the camera's acceleration-induced attitude, as Unity does.
        body=np.eye(3)
        heading=2.*math.atan2(m.pose.pose.orientation.z,m.pose.pose.orientation.w)
        acceleration=np.array([c.acceleration.x,c.acceleration.y,c.acceleration.z]) if latest[0] else np.zeros(3)
        zb=acceleration+np.array([0.,0.,9.81]); zb/=np.linalg.norm(zb)
        yb=np.cross(zb,[math.cos(heading),math.sin(heading),0.]); yb/=np.linalg.norm(yb)
        body=np.column_stack((np.cross(yb,zb),yb,zb))
        from tf.transformations import quaternion_from_matrix
        transform=np.eye(4); transform[:3,:3]=body
        q=quaternion_from_matrix(transform)
        m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=q
        odom.publish(m)
        x,y,vx,vy = car(elapsed)
        p = m.pose.pose.position
        horizontal = math.hypot(p.x-x,p.y-y)
        vertical = abs(p.z-.7)
        separations.append((elapsed,horizontal,vertical,max(horizontal/3.,vertical/1.4)))
        if now-last_cloud > .1:
            cloud.publish(point_cloud2.create_cloud_xyz32(m.header, floor))
            last_cloud = now
        sight=body.T@np.array([x-p.x,y-p.y,.7-p.z])
        visible=sight[0]>.05 and abs(math.atan2(sight[1],sight[0]))<math.radians(37.589) and abs(math.atan2(sight[2],sight[0]))<math.radians(30.)
        target_heading=math.atan2(vy,vx)
        ch,sh=math.cos(target_heading),math.sin(target_heading)
        whole_body_visible=True
        for dx in (-2.2,2.2):
            for dy in (-1.,1.):
                for dz in (-.7,.7):
                    corner=body.T@np.array([x+ch*dx-sh*dy-p.x,
                                           y+sh*dx+ch*dy-p.y,.7+dz-p.z])
                    whole_body_visible &= corner[0]>.05 and abs(math.atan2(corner[1],corner[0]))<math.radians(37.589) and abs(math.atan2(corner[2],corner[0]))<math.radians(30.)
        visibility.append((elapsed,bool(visible),math.atan2(-sight[1],sight[0]),
                           math.atan2(-sight[2],sight[0]),bool(whole_body_visible)))
        if visible or not args.sensor_fov: last_seen=now
        valid=now-last_seen<.65
        if now-last_prediction > .05 and args.prediction_node:
            observation_age.publish(args.observation_delay+.05+now-last_seen)
            target_valid.publish(valid)
            if valid:
                # Match the estimator contract: current-time CV state plus a
                # separate observation age, not a stale odometry header that
                # the predictor correctly rejects at input_timeout.
                obs=Odometry(); obs.header=Header(stamp=stamp,frame_id="world")
                tx,ty,vx,vy=car(max(0.,elapsed-args.observation_delay))
                obs.pose.pose.position.x=tx+vx*args.observation_delay
                obs.pose.pose.position.y=ty+vy*args.observation_delay
                obs.pose.pose.position.z=.7
                obs.pose.pose.orientation.w=1.
                obs.twist.twist.linear.x=vx; obs.twist.twist.linear.y=vy
                obs.pose.covariance[0]=obs.pose.covariance[7]=.2
                if args.noisy_observation:
                    obs.twist.twist.linear.y+=.65*math.sin(5.*elapsed)
                    obs.pose.pose.position.y+=.12*math.sin(3.*elapsed)
                    obs.twist.covariance[0]=obs.twist.covariance[7]=.35
                target_odom.publish(obs)
            last_prediction=now
        if now-last_prediction > .05 and not args.prediction_node:
            path = Path(header=m.header)
            for i in range(11):
                # Current constant-turn model, with no advance knowledge of
                # when the car starts/stops turning.
                x,y,vx,vy = car(elapsed)
                rate = .8 if args.scenario == "uturn" and 7. < elapsed < 7.+math.pi/.8 else 0.
                dt = .25*i
                if rate:
                    x += (math.sin(rate*dt)*vx-(1.-math.cos(rate*dt))*vy)/rate
                    y += ((1.-math.cos(rate*dt))*vx+math.sin(rate*dt)*vy)/rate
                else:
                    x += vx*dt
                    y += vy*dt
                pose = PoseStamped()
                pose.header = Header(stamp=stamp+rospy.Duration(dt),frame_id="world")
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = x,y,.7
                heading = math.atan2(vy,vx)+rate*dt
                pose.pose.orientation.z,pose.pose.orientation.w = math.sin(heading/2),math.cos(heading/2)
                path.poses.append(pose)
            if args.sensor_fov and not valid: path.poses=[]
            target.publish(path)
            last_prediction = now
        time.sleep(.02)
    import rosnode
    assert rosnode.rosnode_ping("/planner_runtime_node", max_count=1, verbose=False), "runtime died"
    norm = lambda values: math.sqrt(sum(v*v for v in values))
    discontinuity = max((norm([b[i]-a[i]-.5*(a[i+3]+b[i+3])*(b[0]-a[0])
                               for i in range(1,4)]) for a,b in zip(commands,commands[1:])
                         if b[0]-a[0] < .15), default=0.)
    result = dict(scenario=args.scenario, commands=len(commands),
                  minimum_horizontal_distance=min(s[1] for s in separations),
                  minimum_separation_ratio=min(s[3] for s in separations),
                  position_discontinuity=discontinuity,
                  max_acceleration=max((norm(c[7:10]) for c in commands),default=0.),
                  max_jerk=max((norm(c[10:13]) for c in commands),default=0.),
                  max_yaw_rate=max((abs(c[14]) for c in commands),default=0.),
                  fresh_commits=sum("elastic_candidate_committed" in e for _,e in events),
                  commits_after_turn=sum(t>14. and "elastic_candidate_committed" in e for t,e in events),
                  emergency=any("emergency" in p for _,p,_ in phases) or
                      any("to=EMER_STOP" in e or "no_safe_recovery" in e for _,e in events),
                  phases=sorted(set(p for _,p,_ in phases)), final_separation=separations[-1],
                  feedback_delay=args.feedback_delay,observation_delay=args.observation_delay,sensor_fov=args.sensor_fov,
                  visible_fraction=sum(v[1] for v in visibility if v[0]>3.)/max(1,sum(v[0]>3. for v in visibility)),
                  desired_distance=desired_distance,
                  final_distance_error=abs(separations[-1][1]-desired_distance),
                  steady_body_visible_fraction=sum(v[4] for v in visibility if v[0]>12.)/max(1,sum(v[0]>12. for v in visibility)),
                  initial_height=args.initial_height,
                  final_height=commands[-1][3] if commands else None,
                  framing_rms_deg=math.degrees(math.sqrt(np.mean([
                      v[2]**2+(v[3]-math.atan2(desired_height_offset,desired_distance))**2 for v in visibility if v[0]>12.]))),
                  minimum_height=min((c[3] for c in commands),default=0.),
                  steady_height_range=(max(c[3] for c in commands if c[0]>15.)-min(c[3] for c in commands if c[0]>15.)) if args.duration>16. else None)
    with open("/tmp/tracking_"+args.scenario+"_trace.json", "w") as out:
        json.dump(dict(result=result, commands=commands, events=events, phases=phases,
                       separations=separations,visibility=visibility), out)
    print(json.dumps(result,indent=2),flush=True)
    assert result["commands"] > 300 and result["fresh_commits"] > 5, "no sustained planning"
    assert result["minimum_separation_ratio"] >= .98, "moving target safety envelope entered"
    assert not result["emergency"], "avoidance failed into emergency stop"
    assert discontinuity < .08, "position command jumped"
    assert result["max_acceleration"] <= 2.1 and result["max_jerk"] <= 6.2, "dynamic limits exceeded"
    assert result["max_yaw_rate"] <= .83, "yaw rate limit exceeded"
    if args.scenario in ("straight","approach"):
        assert result["final_distance_error"] < .8, "tracking distance did not settle"
        assert result["visible_fraction"] > .9, "target repeatedly left the camera"
        assert result["steady_height_range"] < .5, "altitude keeps oscillating"
        assert abs(result["final_height"]-1.2)<.3, "preferred tracking height did not recover"
        assert result["framing_rms_deg"]<12., "camera framing failed to stabilize"
        assert result["steady_body_visible_fraction"]>.9, "vehicle body repeatedly leaves steady camera view"
    if args.scenario == "uturn":
        assert result["commits_after_turn"] >= 5, "tracking did not recover after the turn"
        assert result["final_separation"][1] < 8., "tracking distance did not recover after avoidance"
finally:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
