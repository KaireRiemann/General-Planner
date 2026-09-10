using UnityEngine;
using UnitySensors.Sensor.Camera;

namespace UnitySensors.ROS.Publisher.Image
{
    [RequireComponent(typeof(CameraSensor))]
    public class CameraImageMsgPublisher : ImageMsgPublisher<CameraSensor>
    {
        private CameraSensor _sensor;

        protected override void Start()
        {
            base.Start();
            _sensor = GetComponent<CameraSensor>();
            _sensor.onSensorUpdated += PublishMessage;
        }

        // Publishing from a second timer can serialize a different capture.
        protected override void Update() { }

        private void OnDestroy()
        {
            if (_sensor != null) _sensor.onSensorUpdated -= PublishMessage;
        }
    }
}
