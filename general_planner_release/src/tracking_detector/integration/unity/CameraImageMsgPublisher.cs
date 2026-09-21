using UnityEngine;
using UnitySensors.Sensor.Camera;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

namespace UnitySensors.ROS.Publisher.Image
{
    [RequireComponent(typeof(CameraSensor))]
    public class CameraImageMsgPublisher : ImageMsgPublisher<CameraSensor>
    {
        private CameraSensor _sensor;
        [SerializeField] private string _capturePoseTopic = "";
        private ROSConnection _ros;

        protected override void Start()
        {
            base.Start();
            _sensor = GetComponent<CameraSensor>();
            _ros = ROSConnection.GetOrCreateInstance();
            if (string.IsNullOrEmpty(_capturePoseTopic)) {
                int suffix = TopicName.IndexOf("/color/", System.StringComparison.Ordinal);
                if (suffix < 0) suffix = TopicName.IndexOf("/depth/", System.StringComparison.Ordinal);
                if (suffix < 0) suffix = TopicName.IndexOf("/image", System.StringComparison.Ordinal);
                _capturePoseTopic = (suffix >= 0 ? TopicName.Substring(0,suffix) : TopicName.TrimEnd('/')) + "/capture_pose";
            }
            _ros.RegisterPublisher<PoseStampedMsg>(_capturePoseTopic);
            _sensor.onSensorUpdated += PublishMessage;
        }

        protected override void BeforePublish(Message message)
        {
            RGBCameraSensor rgb = _sensor as RGBCameraSensor;
            if (rgb == null || rgb.CaptureTime <= 0) return;
            HeaderMsg header = null;
            if (message is ImageMsg image) header = image.header;
            if (message is CompressedImageMsg compressed) header = compressed.header;
            if (header == null) return;
            double seconds = System.Math.Floor(rgb.CaptureTime);
            header.stamp = new TimeMsg {
                sec = (uint)seconds,
                nanosec = (uint)((rgb.CaptureTime-seconds)*1e9)
            };
            Vector3 p = rgb.CapturePosition;
            Quaternion q = rgb.CaptureRotation;
            // Unity (x,y,z) -> world (x,z,y); optical axes are right/down/forward.
            Quaternion optical = new Quaternion(-q.x,-q.z,-q.y,q.w) *
                                 Quaternion.Euler(-90f,0f,0f);
            PoseStampedMsg pose = new PoseStampedMsg();
            pose.header.seq = header.seq;
            pose.header.stamp = header.stamp;
            pose.header.frame_id = "world";
            pose.pose.position = new PointMsg(p.x,p.z,p.y);
            pose.pose.orientation = new QuaternionMsg(optical.x,optical.y,optical.z,optical.w);
            _ros.Publish(_capturePoseTopic,pose);
        }

        // Publishing from a second timer can serialize a different capture.
        protected override void Update() { }

        private void OnDestroy()
        {
            if (_sensor != null) _sensor.onSensorUpdated -= PublishMessage;
        }
    }
}
