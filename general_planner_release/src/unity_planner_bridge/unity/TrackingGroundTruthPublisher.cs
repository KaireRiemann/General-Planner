using System;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;

// ROS world (x,y,z) = Unity (x,z,y), matching odom_publisher.cs.
// Publishes the actual car base pose; Odometry twist is in the car frame.
public class TrackingGroundTruthPublisher : MonoBehaviour
{
    private const string TargetName = "Car (1)";
    [SerializeField] private string topic = "/unity/car_ground_truth/odom";
    [SerializeField, Min(1f)] private float publishRate = 20f;
    private ROSConnection ros;
    private Vector3 previousPosition;
    private Quaternion previousRotation;
    private double previousTime = -1.0;
    private double lastPublishTime = -1.0;
    private uint sequence;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Install()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
        SceneManager.sceneLoaded += OnSceneLoaded;
        Attach();
    }

    private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) { Attach(); }

    private static void Attach()
    {
        GameObject car = GameObject.Find(TargetName);
        if (car != null && car.GetComponent("RoadPatrol") != null &&
            car.GetComponent<TrackingGroundTruthPublisher>() == null)
        {
            car.AddComponent<TrackingGroundTruthPublisher>();
            Debug.Log("[tracking ground truth] attached to " + car.name + " in " + car.scene.name);
        }
    }

#if UNITY_EDITOR
    [UnityEditor.InitializeOnLoadMethod]
    private static void AttachAfterReload()
    {
        UnityEditor.EditorApplication.delayCall += () => {
            if (UnityEditor.EditorApplication.isPlaying) Install();
        };
    }
#endif

    private void OnEnable()
    {
        previousTime = lastPublishTime = -1.0;
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topic);
    }

    private void LateUpdate()
    {
        // Sample after RoadPatrol's Update; do not infer motion from its setting.
        double now = Time.timeAsDouble;
        Vector3 position = transform.position;
        Quaternion rotation = transform.rotation;
        double dt = now - previousTime;
        if (previousTime < 0.0 || dt <= 0.0)
        {
            previousPosition = position;
            previousRotation = rotation;
            previousTime = now;
            return;
        }
        Vector3 worldVelocity = (position - previousPosition) / (float)dt;
        Quaternion delta = rotation * Quaternion.Inverse(previousRotation);
        delta.ToAngleAxis(out float angle, out Vector3 axis);
        if (angle > 180f) angle -= 360f;
        Vector3 worldAngularVelocity = Mathf.Abs(angle) < 0.0001f
            ? Vector3.zero : axis * (angle * Mathf.Deg2Rad / (float)dt);
        previousPosition = position;
        previousRotation = rotation;
        previousTime = now;
        if (now - lastPublishTime < 1.0 / Math.Max(1f, publishRate)) return;
        lastPublishTime = now;

        Vector3 localVelocity = transform.InverseTransformDirection(worldVelocity);
        Vector3 localAngularVelocity = transform.InverseTransformDirection(worldAngularVelocity);
        OdometryMsg message = new OdometryMsg();
        message.header.seq = sequence++;
        message.header.frame_id = "world";
        message.header.stamp = new TimeMsg {
            sec = (uint)Math.Floor(now),
            nanosec = (uint)((now - Math.Floor(now)) * 1e9)
        };
        message.child_frame_id = "car_ground_truth";
        message.pose.pose.position = new PointMsg(position.x, position.z, position.y);
        message.pose.pose.orientation = new QuaternionMsg(-rotation.x, -rotation.z, -rotation.y, rotation.w);
        message.twist.twist.linear = new Vector3Msg(localVelocity.x, localVelocity.z, localVelocity.y);
        // Angular velocity is an axial vector under this handedness conversion.
        message.twist.twist.angular = new Vector3Msg(-localAngularVelocity.x, -localAngularVelocity.z, -localAngularVelocity.y);
        ros.Publish(topic, message);
    }
}
