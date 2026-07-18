using System;
using UnityEngine;

namespace CubSim
{
    public readonly struct WheelOdometrySample
    {
        public readonly double Time;
        public readonly double X;
        public readonly double Y;
        public readonly double Yaw;
        public readonly double LinearVelocity;
        public readonly double AngularVelocity;
        public readonly double LeftWheelAngle;
        public readonly double RightWheelAngle;

        public WheelOdometrySample(
            double time,
            double x,
            double y,
            double yaw,
            double linearVelocity,
            double angularVelocity,
            double leftWheelAngle,
            double rightWheelAngle)
        {
            Time = time;
            X = x;
            Y = y;
            Yaw = yaw;
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;
            LeftWheelAngle = leftWheelAngle;
            RightWheelAngle = rightWheelAngle;
        }
    }

    public sealed class McubRobot : MonoBehaviour
    {
        private const float CommandTimeoutSeconds = 0.5f;
        private const float MaximumMotorTorque = 1.0f;
        private const float VelocityGain = 0.16f;

        private WheelCollider leftWheel;
        private WheelCollider rightWheel;
        private Transform leftVisual;
        private Transform rightVisual;
        private float wheelRadius;
        private float wheelTrack;
        private float targetLinear;
        private float targetAngular;
        private float lastCommandTime = float.NegativeInfinity;
        private double odomX;
        private double odomY;
        private double odomYaw;
        private double leftWheelAngle;
        private double rightWheelAngle;

        public Rigidbody Body { get; private set; }
        public float WheelRadius => wheelRadius;
        public float WheelTrack => wheelTrack;
        public WheelOdometrySample Odometry { get; private set; }

        public void Configure(
            Rigidbody body,
            WheelCollider left,
            WheelCollider right,
            Transform leftWheelVisual,
            Transform rightWheelVisual,
            float radius,
            float track)
        {
            Body = body;
            leftWheel = left;
            rightWheel = right;
            leftVisual = leftWheelVisual;
            rightVisual = rightWheelVisual;
            wheelRadius = radius;
            wheelTrack = track;
        }

        public void SetVelocityCommand(float linearMetersPerSecond, float angularRadiansPerSecond)
        {
            targetLinear = Mathf.Clamp(linearMetersPerSecond, -1f, 1f);
            targetAngular = Mathf.Clamp(angularRadiansPerSecond, -4f, 4f);
            lastCommandTime = Time.time;
        }

        private void FixedUpdate()
        {
            if (leftWheel == null || rightWheel == null)
            {
                return;
            }

            if (Time.time - lastCommandTime > CommandTimeoutSeconds)
            {
                targetLinear = 0f;
                targetAngular = 0f;
            }

            var leftTarget = (targetLinear - targetAngular * wheelTrack * 0.5f) / wheelRadius;
            var rightTarget = (targetLinear + targetAngular * wheelTrack * 0.5f) / wheelRadius;
            ApplyMotor(leftWheel, leftTarget);
            ApplyMotor(rightWheel, rightTarget);
            UpdateWheelVisual(leftWheel, leftVisual);
            UpdateWheelVisual(rightWheel, rightVisual);
            UpdateOdometry(Time.fixedDeltaTime);
        }

        private static void ApplyMotor(WheelCollider wheel, float targetRadiansPerSecond)
        {
            var measured = wheel.rpm * Mathf.PI / 30f;
            var torque = Mathf.Clamp((targetRadiansPerSecond - measured) * VelocityGain, -MaximumMotorTorque, MaximumMotorTorque);
            wheel.motorTorque = torque;
            wheel.brakeTorque = Mathf.Abs(targetRadiansPerSecond) < 0.01f ? 0.08f : 0f;
        }

        private static void UpdateWheelVisual(WheelCollider wheel, Transform visual)
        {
            if (visual == null)
            {
                return;
            }

            wheel.GetWorldPose(out var position, out var rotation);
            visual.position = position;
            visual.rotation = rotation * Quaternion.Euler(0f, 0f, 90f);
        }

        private void UpdateOdometry(float deltaTime)
        {
            var leftOmega = leftWheel.rpm * Math.PI / 30.0;
            var rightOmega = rightWheel.rpm * Math.PI / 30.0;
            leftWheelAngle += leftOmega * deltaTime;
            rightWheelAngle += rightOmega * deltaTime;

            var leftDistance = leftOmega * wheelRadius * deltaTime;
            var rightDistance = rightOmega * wheelRadius * deltaTime;
            var distance = (leftDistance + rightDistance) * 0.5;
            var yawDelta = (rightDistance - leftDistance) / wheelTrack;
            var midYaw = odomYaw + yawDelta * 0.5;
            odomX += distance * Math.Cos(midYaw);
            odomY += distance * Math.Sin(midYaw);
            odomYaw = NormalizeAngle(odomYaw + yawDelta);

            Odometry = new WheelOdometrySample(
                Time.timeAsDouble,
                odomX,
                odomY,
                odomYaw,
                distance / deltaTime,
                yawDelta / deltaTime,
                leftWheelAngle,
                rightWheelAngle);
        }

        private static double NormalizeAngle(double angle)
        {
            while (angle > Math.PI) angle -= Math.PI * 2.0;
            while (angle < -Math.PI) angle += Math.PI * 2.0;
            return angle;
        }
    }
}
