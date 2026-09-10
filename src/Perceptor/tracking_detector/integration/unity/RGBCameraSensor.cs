namespace UnitySensors.Sensor.Camera
{
    public class RGBCameraSensor : CameraSensor
    {
        protected override void Init()
        {
            base.Init();
            // Render only at acquisition, using the pose for sensor.time.
            m_camera.enabled = false;
        }

        protected override void UpdateSensor()
        {
            m_camera.Render();
            base.UpdateSensor();
        }
    }
}
