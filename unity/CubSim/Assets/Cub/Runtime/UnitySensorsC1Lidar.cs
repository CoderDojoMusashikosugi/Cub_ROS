using System;
using Unity.Mathematics;
using UnityEngine;
using UnitySensors.Sensor.LiDAR;

namespace CubSim
{
    [RequireComponent(typeof(RaycastLiDARSensor))]
    public sealed class UnitySensorsC1Lidar : MonoBehaviour, IC1ScanSource
    {
        private RaycastLiDARSensor sensor;

        public C1Scan LatestScan { get; private set; }
        public event Action<C1Scan> ScanReady;

        private void OnEnable()
        {
            sensor = GetComponent<RaycastLiDARSensor>();
            sensor.onSensorUpdateComplete += OnSensorUpdated;
        }

        private void OnDisable()
        {
            if (sensor != null)
            {
                sensor.onSensorUpdateComplete -= OnSensorUpdated;
            }
        }

        private void OnSensorUpdated()
        {
            var pointCloud = sensor.pointCloud;
            if (pointCloud == null || !pointCloud.points.IsCreated)
            {
                return;
            }

            var points = pointCloud.points;
            var ranges = new float[points.Length];
            var intensities = new float[points.Length];
            for (var index = 0; index < points.Length; index++)
            {
                float3 position = points[index].position;
                var range = Mathf.Sqrt(position.x * position.x + position.z * position.z);
                ranges[index] = range > 0f ? range : float.PositiveInfinity;
                intensities[index] = points[index].intensity;
            }

            LatestScan = new C1Scan(sensor.time, ranges, intensities);
            ScanReady?.Invoke(LatestScan);
        }
    }
}
