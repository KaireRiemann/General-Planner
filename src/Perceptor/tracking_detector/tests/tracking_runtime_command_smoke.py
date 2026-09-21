#!/usr/bin/env python3
"""Synthetic runtime smoke on an owned master; never drives the user's vehicle."""
import os,sys,time,subprocess,signal,json,math
os.environ["ROS_MASTER_URI"]="http://127.0.0.1:11329"
import rosgraph
try: rosgraph.Master("/probe").getPid()
except Exception: pass
else: raise RuntimeError("isolated port 11329 already in use")
mode="state2state" if "--state2state" in sys.argv else "tracking"
reacquire="--reacquire" in sys.argv and mode=="tracking"
moving_target=("--moving-target" in sys.argv or reacquire) and mode=="tracking"
processes=[]
def spawn(args,name):
    log=open("/tmp/tracking_runtime_smoke_"+name+".log","w")
    p=subprocess.Popen(args,stdout=log,stderr=log,start_new_session=True)
    processes.append(p)
    return p
try:
    spawn(["roscore","-p","11329"],"master")
    for _ in range(60):
        try:rosgraph.Master("/probe").getPid();break
        except Exception:time.sleep(.1)
    import rospy
    from nav_msgs.msg import Odometry,Path
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Header,String
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from quadrotor_msgs.msg import PositionCommand
    from general_planner.msg import PlannerStatus
    from rosgraph_msgs.msg import Log
    rospy.init_node("tracking_runtime_command_smoke")
    package="general_planner_release" if "--release" in sys.argv else "task_planner"
    spawn(["roslaunch",package,"planner_runtime.launch","initial_mode:="+mode,
           "tracking_detector:=false","perceptor:=false","rviz:=false",
           "auto_rviz_switch:=false","enable_exploration:=false"],"runtime")
    commands=[];states=[];events=[];diagnostics=[];latest=[None]
    def on_command(m):
        latest[0]=m
        commands.append([time.monotonic(),m.position.x,m.position.y,m.position.z,
                         m.velocity.x,m.velocity.y,m.velocity.z,
                         m.acceleration.x,m.acceleration.y,m.acceleration.z,
                         m.jerk.x,m.jerk.y,m.jerk.z])
    rospy.Subscriber("/planning/pos_cmd",PositionCommand,on_command)
    rospy.Subscriber("/planner/status",PlannerStatus,lambda m:states.append((time.monotonic(),m.phase_str,m.reason)))
    rospy.Subscriber("/rosout_agg",Log,lambda m:events.append((time.monotonic(),m.msg)))
    rospy.Subscriber("/planning/diagnostics/events",String,lambda m:diagnostics.append((time.monotonic(),m.data)))
    odom=rospy.Publisher("/lidar_slam/odom",Odometry,queue_size=1)
    cloud=rospy.Publisher("/cloud_registered",PointCloud2,queue_size=1)
    target=rospy.Publisher("/tracking/target_prediction",Path,queue_size=1)
    raw_target=rospy.Publisher("/target_ekf_node/target_odom" if "--release" in sys.argv else "/tracking/target_odom",
                               Odometry,queue_size=1)
    goal_pub=rospy.Publisher("/goal_3d",PoseStamped,queue_size=1)
    sent_goal=False
    floor=[[i*.4,j*.4,0.] for i in range(-15,126 if reacquire else 36) for j in range(-15,16)]
    start=time.monotonic(); last_cloud=0.;moving_since=None;previous_lost=False
    while time.monotonic()-start<30:
        now=time.monotonic();stamp=rospy.Time.now()
        m=Odometry();m.header.stamp=stamp;m.header.frame_id="world"
        m.pose.pose.position.z=1.5;m.pose.pose.orientation.w=1.
        if reacquire:
            m.pose.pose.position.y=2.5
            m.pose.pose.orientation.z=math.sin(.2);m.pose.pose.orientation.w=math.cos(.2)
        if latest[0]:
            c=latest[0];m.pose.pose.position=c.position
            m.twist.twist.linear=c.velocity
            m.pose.pose.orientation.z=math.sin(c.yaw/2);m.pose.pose.orientation.w=math.cos(c.yaw/2)
            if math.sqrt(c.velocity.x**2+c.velocity.y**2+c.velocity.z**2)>.2 and moving_since is None:moving_since=now
        odom.publish(m)
        if mode=="state2state" and not sent_goal and now-start>2 and states and states[-1][1]=="waiting_input":
            goal=PoseStamped();goal.header=m.header;goal.pose.position.x=6.
            goal.pose.position.z=1.5;goal.pose.orientation.w=1.
            goal_pub.publish(goal);sent_goal=True
        if now-last_cloud>.1:
            cloud.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id="world"),floor));last_cloud=now
        path=Path();path.header=m.header
        lost=mode=="tracking" and moving_since is not None and now-moving_since>(6.0 if moving_target else 1.5)
        if reacquire:
            motion_age=now-moving_since if moving_since else 0.
            lost=3.0<motion_age<4.0 or motion_age>13.
        if not lost:
            for i in range(4 if reacquire else 7):
                pose=PoseStamped();pose.header.stamp=stamp+rospy.Duration(i*.25)
                pose.header.frame_id="world";pose.pose.position.x=8.;pose.pose.position.z=.7;pose.pose.orientation.w=1
                if moving_target:
                    target_t=max(0.,now-start-2)+i*.25
                    pose.pose.position.x=8.+.5*target_t
                    pose.pose.position.y=1.5*math.sin(.15*target_t)
                    if reacquire:
                        pose.pose.position.x=13.+2.*target_t
                        pose.pose.position.y=0.
                path.poses.append(pose)
        if reacquire:
            # Raw odometry must not resurrect a prediction explicitly cleared
            # by its authoritative producer, even after the producer goes quiet.
            raw=Odometry();raw.header=m.header;raw.pose.pose.orientation.w=1.
            raw.pose.pose.position.x=13.+2.*max(0.,now-start-2.)
            raw.pose.pose.position.z=.7;raw.twist.twist.linear.x=2.
            raw_target.publish(raw)
        if not reacquire or not lost or not previous_lost:target.publish(path)
        previous_lost=lost
        if moving_since and now-moving_since>(17.0 if reacquire else 11.0 if moving_target else 7.0):break
        time.sleep(.02)
    import rosnode
    assert rosnode.rosnode_ping("/planner_runtime_node",max_count=1,verbose=False),"runtime node died"
    failures=[msg for t,msg in events if "source timeout" in msg and moving_since and t>moving_since]
    result=dict(mode=mode,moved=moving_since is not None,commands=len(commands),
                timeouts_after_motion=failures,phases=sorted(set(s[1] for s in states)),
                tail_states=states[-5:],events=[(t,m) for t,m in events if "TRACKING_HOLD" in m or "Tracking task success" in m])
    if moving_since:
        times=[c[0] for c in commands if c[0]>moving_since]
        result["max_command_gap"]=max((b-a for a,b in zip(times,times[1:])),default=0)
    if commands:
        result["max_acceleration"]=max(math.sqrt(sum(x*x for x in c[7:10])) for c in commands)
        result["max_jerk"]=max(math.sqrt(sum(x*x for x in c[10:13])) for c in commands)
        result["max_acceleration_step"]=max((math.sqrt(sum((b[i]-a[i])**2 for i in range(7,10))) for a,b in zip(commands,commands[1:])),default=0)
        result["terminal_speed"]=math.sqrt(sum(x*x for x in commands[-1][4:7]))
        moving_commands=[c for c in commands if moving_since is not None and c[0]>=moving_since]
        result["max_position_step"]=max((math.sqrt(sum((b[i]-a[i])**2 for i in range(1,4)))
                                         for a,b in zip(moving_commands,moving_commands[1:])),default=0)

    if moving_target:
        result["unexpected_recovery_holds"]=[m for t,m in events
            if "TRACKING_HOLD_COMMITTED" in m and "tracking fallback:" in m]
    if reacquire:
        result["fresh_commits_after_reacquisition"]=sum("elastic_candidate_committed" in m and
            moving_since is not None and 4.3<t-moving_since<13. for t,m in diagnostics)
        if moving_since:
            settled=min(commands,key=lambda c:abs(c[0]-moving_since-12.5))
            target_x=13.+2.*max(0.,settled[0]-start-2.)
            result["pre_loss_tracking_distance"]=math.hypot(target_x-settled[1],settled[2])
            result["pre_loss_velocity_error"]=math.sqrt((settled[4]-2.)**2+settled[5]**2+settled[6]**2)
    with open("/tmp/"+mode+"_runtime_command_smoke.json","w") as f:json.dump(result,f,indent=2)
    with open("/tmp/"+mode+"_runtime_trace.json","w") as f:json.dump(dict(start=start,moving_since=moving_since,commands=commands,events=events,states=states),f)
    print(json.dumps(result,indent=2),flush=True)
    if not result["moved"] or failures:sys.exit(1)
    if moving_target and result["unexpected_recovery_holds"]:sys.exit(1)
    if reacquire and result["fresh_commits_after_reacquisition"]<5:sys.exit(1)
    if reacquire and (result["pre_loss_tracking_distance"]>8. or result["pre_loss_velocity_error"]>.5):sys.exit(1)
    if mode=="tracking" and ("braking" not in result["phases"] or result["max_acceleration"]>3.2 or result["max_jerk"]>13 or result["max_acceleration_step"]>.8 or result["terminal_speed"]>.01 or result["max_position_step"]>.15):sys.exit(1)
finally:
    for p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
