#!/usr/bin/env python3
"""Owns an isolated master; never publishes on the user's running master."""
import os,sys,time,signal,subprocess,json,math
from pathlib import Path
import numpy as np
os.environ["ROS_MASTER_URI"]="http://127.0.0.1:11329"
import rosgraph
try: rosgraph.Master("/probe").getPid()
except Exception: pass
else: raise RuntimeError("11329 already in use")
root=Path(__file__).resolve().parents[1]
processes=[]
def spawn(cmd):
    log=open("/tmp/estimator_test_%d.log"%len(processes),"w")
    p=subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True)
    processes.append(p)
    return p
try:
    spawn(["roscore","-p","11329"])
    for _ in range(50):
        try:rosgraph.Master("/probe").getPid();break
        except Exception:time.sleep(.1)
    import rospy
    from nav_msgs.msg import Odometry,Path as Prediction
    from sensor_msgs.msg import CameraInfo
    from tracking_detector.msg import BoundingBoxes,BoundingBox
    from std_msgs.msg import Bool,String
    rospy.init_node("estimator_regression")
    spawn(["python3",str(root/"scripts/target_state_estimator.py"),
           "__name:=estimate","~odom:=/input/odom","~yolo:=/input/boxes",
           "~camera_info:=/input/info","~valid:=/tracking/target_valid",
           "~observation_age:=/tracking/observation_age","~target_odom:=/target",
           "~status:=/tracking/status"])
    if "--predictor" in sys.argv:
        spawn(["rosrun","tracking_detector","target_path_predictor_node",
               "__name:=predict","_require_target_valid:=true","_prediction_horizon:=1.5",
               "~target_odom:=/target","~target_valid:=/tracking/target_valid",
               "~observation_age:=/tracking/observation_age","~prediction:=/prediction"])
    outputs=[];states=[];paths=[]
    rospy.Subscriber("/target",Odometry,lambda m:outputs.append(m))
    rospy.Subscriber("/tracking/status",String,lambda m:states.append(json.loads(m.data)))
    rospy.Subscriber("/prediction",Prediction,lambda m:paths.append(m))
    odom=rospy.Publisher("/input/odom",Odometry,queue_size=30)
    boxes=rospy.Publisher("/input/boxes",BoundingBoxes,queue_size=2)
    info=rospy.Publisher("/input/info",CameraInfo,queue_size=1,latch=True)
    K=np.array([[415.6922,0,320],[0,415.6922,240],[0,0,1]])
    camera=CameraInfo();camera.width=640;camera.height=480;camera.K=K.reshape(-1).tolist()
    start=time.monotonic()
    history=[]
    def frame(detect=True, calibrate=True):
        elapsed=time.monotonic()-start;stamp=100+elapsed
        yaw=.18*math.sin(elapsed)
        body=np.array([.3*math.sin(elapsed*.5),0,1.5])
        R=np.array([[math.cos(yaw),-math.sin(yaw),0],
                    [math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        Rcb=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        optical=(R@Rcb).T@(np.array([8.,0,0])-body-np.array([0,0,.1]))
        u=K[0,0]*optical[0]/optical[2]+320
        bottom=K[1,1]*optical[1]/optical[2]+240
        m=Odometry();m.header.stamp=rospy.Time.from_sec(stamp);m.header.frame_id="world"
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=body
        m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2)
        odom.publish(m)
        if calibrate:info.publish(camera)
        b=BoundingBoxes();b.header=m.header;b.image_header=m.header
        if detect:
            box=BoundingBox();box.Class="car";box.probability=.9
            box.xmin=int(u-30);box.xmax=int(u+30);box.ymin=int(bottom-45);box.ymax=int(bottom)
            b.bounding_boxes=[box]
        history.append((time.monotonic(),b))
        # Delayed detector output matches historical, not current, ego pose.
        while history and time.monotonic()-history[0][0]>.15:
            boxes.publish(history.pop(0)[1])
        time.sleep(.05)
    for _ in range(20):frame(calibrate=False)
    assert not outputs,"must wait for calibration"
    for _ in range(110):frame()
    assert len(outputs)>30,states[-5:]
    p=np.array([[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z] for m in outputs[-50:]])
    error=np.linalg.norm(p-np.array([8,0,.7]),axis=1)
    assert max(error)<.25,(max(error),p[-1])
    print("PASS moving-camera stationary-target maximum error",max(error),flush=True)
    if "--predictor" in sys.argv:
        assert all(len(m.poses)>=4 for m in paths if m.poses), "prediction horizon collapsed below 0.75s"
    for _ in range(30):frame(detect=False)
    assert states[-1]["state"]=="lost",states[-1]
    count=len(outputs)
    for _ in range(10):frame(detect=False)
    assert len(outputs)==count,"lost track must stop publishing odometry"
    if "--predictor" in sys.argv:
        assert paths and not paths[-1].poses,"loss must clear prediction"
    for _ in range(35):frame()
    assert len(outputs)>count and states[-1]["valid"],states[-1]
    assert states[-1]["track_id"]==2,states[-1]
    if "--predictor" in sys.argv:
        assert paths[-1].poses and 2<=len(paths[-1].poses)<=7
    print("PASS loss, explicit invalidation, confirmed reacquisition",flush=True)
finally:
    for p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
