using System;
using UnityEngine;

namespace CubSim
{
    public readonly struct C1Scan
    {
        public readonly double Time;
        public readonly float AngleMin;
        public readonly float AngleMax;
        public readonly float AngleIncrement;
        public readonly float TimeIncrement;
        public readonly float ScanTime;
        public readonly float RangeMin;
        public readonly float RangeMax;
        public readonly float[] Ranges;
        public readonly float[] Intensities;

        public C1Scan(double time, float[] ranges, float[] intensities)
        {
            Time = time;
            AngleMin = -Mathf.PI;
            AngleIncrement = Mathf.Deg2Rad * 0.72f;
            AngleMax = AngleMin + AngleIncrement * (ranges.Length - 1);
            ScanTime = 0.1f;
            TimeIncrement = 1f / 5000f;
            RangeMin = 0.05f;
            RangeMax = 12f;
            Ranges = ranges;
            Intensities = intensities;
        }
    }

    public sealed class LidarReflectivity : MonoBehaviour
    {
        [Range(0f, 1f)] public float Value = 0.7f;
    }

    public sealed class SlamtecC1Lidar : MonoBehaviour, IC1ScanSource
    {
        public const int SamplesPerScan = 500;
        public const float ScanFrequencyHz = 10f;
        public const float SampleFrequencyHz = 5000f;
        public const float MinimumRangeMeters = 0.05f;
        public const float WhiteTargetRangeMeters = 12f;
        public const float BlackTargetRangeMeters = 6f;
        public const float AngularResolutionDegrees = 0.72f;
        public const float RangeResolutionMeters = 0.015f;
        public const float MaximumAccuracyErrorMeters = 0.03f;

        [SerializeField] private bool addMeasurementNoise = true;
        [SerializeField] private bool drawDebugRays;
        [SerializeField] private int randomSeed = 323;

        private System.Random random;
        private float nextScanTime;

        public C1Scan LatestScan { get; private set; }
        public event Action<C1Scan> ScanReady;

        private void Awake()
        {
            random = new System.Random(randomSeed);
            nextScanTime = Time.time;
        }

        private void FixedUpdate()
        {
            if (Time.time + 1e-6f < nextScanTime)
            {
                return;
            }

            do
            {
                nextScanTime += 1f / ScanFrequencyHz;
            } while (nextScanTime <= Time.time);

            Measure();
        }

        public C1Scan Measure()
        {
            var ranges = new float[SamplesPerScan];
            var intensities = new float[SamplesPerScan];
            var angleMin = -Mathf.PI;
            var angleIncrement = Mathf.Deg2Rad * AngularResolutionDegrees;
            for (var index = 0; index < SamplesPerScan; index++)
            {
                var angle = angleMin + index * angleIncrement;
                var localDirection = new Vector3(-Mathf.Sin(angle), 0f, Mathf.Cos(angle));
                var direction = transform.TransformDirection(localDirection);
                if (Physics.Raycast(
                        transform.position,
                        direction,
                        out var hit,
                        WhiteTargetRangeMeters,
                        Physics.DefaultRaycastLayers,
                        QueryTriggerInteraction.Ignore))
                {
                    var reflectivity = hit.collider.GetComponentInParent<LidarReflectivity>();
                    var reflectivityValue = reflectivity == null ? 0.7f : Mathf.Clamp01(reflectivity.Value);
                    var targetRange = Mathf.Lerp(BlackTargetRangeMeters, WhiteTargetRangeMeters, reflectivityValue);
                    var measuredRange = hit.distance;
                    if (measuredRange >= MinimumRangeMeters && measuredRange <= targetRange)
                    {
                        if (addMeasurementNoise)
                        {
                            var error = Mathf.Clamp((float)NextGaussian() * 0.01f, -MaximumAccuracyErrorMeters, MaximumAccuracyErrorMeters);
                            measuredRange += error;
                        }

                        measuredRange = Mathf.Round(measuredRange / RangeResolutionMeters) * RangeResolutionMeters;
                        ranges[index] = measuredRange;
                        intensities[index] = Mathf.Clamp(255f * reflectivityValue / (1f + measuredRange * 0.1f), 0f, 255f);
                        if (drawDebugRays) Debug.DrawRay(transform.position, direction * measuredRange, Color.green, 0.1f);
                        continue;
                    }
                }

                ranges[index] = float.PositiveInfinity;
                intensities[index] = 0f;
                if (drawDebugRays) Debug.DrawRay(transform.position, direction * WhiteTargetRangeMeters, Color.red, 0.1f);
            }

            LatestScan = new C1Scan(Time.timeAsDouble, ranges, intensities);
            ScanReady?.Invoke(LatestScan);
            return LatestScan;
        }

        private double NextGaussian()
        {
            // Enter Play Mode can reload serialized MonoBehaviours without calling Awake.
            // System.Random is not serialized, so restore it lazily as well.
            if (random == null) random = new System.Random(randomSeed);
            var first = 1.0 - random.NextDouble();
            var second = 1.0 - random.NextDouble();
            return Math.Sqrt(-2.0 * Math.Log(first)) * Math.Sin(2.0 * Math.PI * second);
        }
    }
}
