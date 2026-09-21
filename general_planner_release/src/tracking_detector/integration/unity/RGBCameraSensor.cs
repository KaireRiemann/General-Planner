namespace UnitySensors.Sensor.Camera
{
    public class RGBCameraSensor : CameraSensor
    {
        public double CaptureTime { get; private set; }
        public UnityEngine.Vector3 CapturePosition { get; private set; }
        public UnityEngine.Quaternion CaptureRotation { get; private set; }
        protected override void Init()
        {
            base.Init();
            // Render only at acquisition, using the pose for sensor.time.
            m_camera.enabled = false;
        }

        protected override void UpdateSensor()
        {
            CaptureTime = UnityEngine.Time.timeAsDouble;
            CapturePosition = m_camera.transform.position;
            CaptureRotation = m_camera.transform.rotation;
            m_camera.Render();
            base.UpdateSensor();
        }
    }
}
