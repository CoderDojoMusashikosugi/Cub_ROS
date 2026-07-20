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
        private const float VelocityGain = 0.50f;
        private const float MaximumLinearAcceleration = 0.75f;
        private const float MaximumLinearDeceleration = 1.5f;
        private const float MaximumAngularCommand = 4f;
        private const float ReferenceManualAngularCommand = Mathf.PI / 3f;
        private const float RequiredAngularResponseTime = 0.3f;
        private const float AngularActuatorAcceleration =
            ReferenceManualAngularCommand / RequiredAngularResponseTime;
        // Extra iterations are required for stable velocity drives under wheel/ground contact.
        private const int ArticulationSolverIterations = 20;
        private const int ArticulationSolverVelocityIterations = 5;

        private WheelCollider leftWheel;
        private WheelCollider rightWheel;
        [SerializeField] private ArticulationBody rootArticulation;
        [SerializeField] private ArticulationBody leftArticulation;
        [SerializeField] private ArticulationBody rightArticulation;
        private Transform leftVisual;
        private Transform rightVisual;
        [SerializeField] private float wheelRadius;
        [SerializeField] private float wheelTrack;
        [SerializeField] private float maximumMotorTorque;
        private float targetLinear;
        private float targetAngular;
        private float commandedLinear;
        private float commandedAngular;
        private float lastCommandTime = float.NegativeInfinity;
        private double odomX;
        private double odomY;
        private double odomYaw;
        private double leftWheelAngle;
        private double rightWheelAngle;
        private PhysicsMaterial runtimeDriveWheelMaterial;
        private PhysicsMaterial runtimeCasterMaterial;

        public Rigidbody Body { get; private set; }
        public ArticulationBody RootArticulation => rootArticulation;
        public float WheelRadius => wheelRadius;
        public float WheelTrack => wheelTrack;
        public float MaximumMotorTorque => maximumMotorTorque;
        public WheelOdometrySample Odometry { get; private set; }

        private void Awake()
        {
            if (rootArticulation == null || leftArticulation == null || rightArticulation == null)
            {
                return;
            }

            foreach (var body in rootArticulation.GetComponentsInChildren<ArticulationBody>(true))
            {
                body.solverIterations = ArticulationSolverIterations;
                body.solverVelocityIterations = ArticulationSolverVelocityIterations;
            }

            ConfigureVelocityDrive(leftArticulation, maximumMotorTorque);
            ConfigureVelocityDrive(rightArticulation, maximumMotorTorque);

            var centerOfMass = rootArticulation.centerOfMass;
            centerOfMass.y = Mathf.Min(centerOfMass.y, 0.04f);
            rootArticulation.centerOfMass = centerOfMass;

            runtimeDriveWheelMaterial = CreateRuntimeContactMaterial("mCub drive wheel", 1.20f, 1.00f, PhysicsMaterialCombine.Maximum);
            runtimeCasterMaterial = CreateRuntimeContactMaterial("mCub caster", 0f, 0f, PhysicsMaterialCombine.Minimum);
            AssignContactMaterial(leftArticulation.transform, runtimeDriveWheelMaterial);
            AssignContactMaterial(rightArticulation.transform, runtimeDriveWheelMaterial);
            foreach (var item in transform.GetComponentsInChildren<Transform>(true))
            {
                if (item.name == "caster_wheel_link")
                {
                    AssignContactMaterial(item, runtimeCasterMaterial);
                    break;
                }
            }
        }

        public void Configure(
            Rigidbody body,
            WheelCollider left,
            WheelCollider right,
            Transform leftWheelVisual,
            Transform rightWheelVisual,
            float radius,
            float track,
            float motorTorque)
        {
            Body = body;
            leftWheel = left;
            rightWheel = right;
            leftVisual = leftWheelVisual;
            rightVisual = rightWheelVisual;
            wheelRadius = radius;
            wheelTrack = track;
            maximumMotorTorque = PositiveMotorTorque(motorTorque);
        }

        public void ConfigureArticulation(
            ArticulationBody root,
            ArticulationBody left,
            ArticulationBody right,
            float radius,
            float track,
            float motorTorque)
        {
            rootArticulation = root ?? throw new ArgumentNullException(nameof(root));
            leftArticulation = left ?? throw new ArgumentNullException(nameof(left));
            rightArticulation = right ?? throw new ArgumentNullException(nameof(right));
            wheelRadius = radius;
            wheelTrack = track;
            maximumMotorTorque = PositiveMotorTorque(motorTorque);
            ConfigureVelocityDrive(leftArticulation, maximumMotorTorque);
            ConfigureVelocityDrive(rightArticulation, maximumMotorTorque);
        }

        private static float PositiveMotorTorque(float motorTorque)
        {
            if (motorTorque <= 0f)
            {
                throw new ArgumentOutOfRangeException(nameof(motorTorque), "URDF wheel effort must be positive.");
            }

            return motorTorque;
        }

        private static void ConfigureVelocityDrive(ArticulationBody joint, float motorTorque)
        {
            var drive = joint.xDrive;
            drive.stiffness = 0f;
            drive.damping = VelocityGain;
            drive.forceLimit = motorTorque;
            joint.xDrive = drive;
            joint.maxAngularVelocity = 30f;
        }

        public void SetVelocityCommand(float linearMetersPerSecond, float angularRadiansPerSecond)
        {
            targetLinear = Mathf.Clamp(linearMetersPerSecond, -1f, 1f);
            targetAngular = Mathf.Clamp(angularRadiansPerSecond, -MaximumAngularCommand, MaximumAngularCommand);
            lastCommandTime = Time.time;
        }

        private void FixedUpdate()
        {
            if (leftArticulation != null && rightArticulation != null)
            {
                FixedUpdateArticulation();
                return;
            }

            if (leftWheel == null || rightWheel == null)
            {
                return;
            }

            if (Time.time - lastCommandTime > CommandTimeoutSeconds)
            {
                targetLinear = 0f;
                targetAngular = 0f;
            }

            UpdateAppliedCommand(Time.fixedDeltaTime);
            var leftTarget = (commandedLinear - commandedAngular * wheelTrack * 0.5f) / wheelRadius;
            var rightTarget = (commandedLinear + commandedAngular * wheelTrack * 0.5f) / wheelRadius;
            ApplyMotor(leftWheel, leftTarget);
            ApplyMotor(rightWheel, rightTarget);
            UpdateWheelVisual(leftWheel, leftVisual);
            UpdateWheelVisual(rightWheel, rightVisual);
            UpdateOdometry(Time.fixedDeltaTime, leftWheel.rpm * Math.PI / 30.0, rightWheel.rpm * Math.PI / 30.0);
        }

        private void ApplyMotor(WheelCollider wheel, float targetRadiansPerSecond)
        {
            var measured = wheel.rpm * Mathf.PI / 30f;
            var torque = Mathf.Clamp((targetRadiansPerSecond - measured) * VelocityGain, -maximumMotorTorque, maximumMotorTorque);
            wheel.motorTorque = torque;
            wheel.brakeTorque = Mathf.Abs(targetRadiansPerSecond) < 0.01f ? 0.08f : 0f;
        }

        private void FixedUpdateArticulation()
        {
            if (Time.time - lastCommandTime > CommandTimeoutSeconds)
            {
                targetLinear = 0f;
                targetAngular = 0f;
            }

            UpdateAppliedCommand(Time.fixedDeltaTime);
            ApplyYawActuator(Time.fixedDeltaTime);
            var leftTarget = (commandedLinear - commandedAngular * wheelTrack * 0.5f) / wheelRadius;
            var rightTarget = (commandedLinear + commandedAngular * wheelTrack * 0.5f) / wheelRadius;
            // ArticulationDrive expresses revolute joint velocity in degrees per second.
            leftArticulation.SetDriveTargetVelocity(ArticulationDriveAxis.X, leftTarget * Mathf.Rad2Deg);
            rightArticulation.SetDriveTargetVelocity(ArticulationDriveAxis.X, rightTarget * Mathf.Rad2Deg);
            UpdateOdometry(
                Time.fixedDeltaTime,
                JointVelocity(leftArticulation),
                JointVelocity(rightArticulation));
        }

        private void ApplyYawActuator(float deltaTime)
        {
            // PhysX wheel contact under-transmits the closed-loop motor response to body yaw.
            // Correct the measured yaw rate using the response time observed on the real mCub.
            var angularVelocity = rootArticulation.angularVelocity;
            angularVelocity.y = Mathf.MoveTowards(
                angularVelocity.y,
                commandedAngular,
                AngularActuatorAcceleration * deltaTime);
            rootArticulation.angularVelocity = angularVelocity;
        }

        private static PhysicsMaterial CreateRuntimeContactMaterial(
            string name,
            float staticFriction,
            float dynamicFriction,
            PhysicsMaterialCombine frictionCombine)
        {
            return new PhysicsMaterial(name)
            {
                staticFriction = staticFriction,
                dynamicFriction = dynamicFriction,
                bounciness = 0f,
                frictionCombine = frictionCombine,
                bounceCombine = PhysicsMaterialCombine.Minimum,
                hideFlags = HideFlags.DontSave
            };
        }

        private static void AssignContactMaterial(Transform link, PhysicsMaterial material)
        {
            foreach (var collider in link.GetComponentsInChildren<Collider>(true))
            {
                collider.sharedMaterial = material;
            }
        }

        private void OnDestroy()
        {
            if (runtimeDriveWheelMaterial != null) Destroy(runtimeDriveWheelMaterial);
            if (runtimeCasterMaterial != null) Destroy(runtimeCasterMaterial);
        }

        private void UpdateAppliedCommand(float deltaTime)
        {
            var linearRate = Mathf.Abs(targetLinear) < Mathf.Abs(commandedLinear)
                ? MaximumLinearDeceleration
                : MaximumLinearAcceleration;
            commandedLinear = Mathf.MoveTowards(commandedLinear, targetLinear, linearRate * deltaTime);
            // The real motor controller receives angular velocity changes immediately.
            // Let motor effort, wheel contact, and body inertia determine yaw acceleration.
            commandedAngular = targetAngular;
        }

        private static double JointVelocity(ArticulationBody joint)
        {
            var velocity = joint.jointVelocity;
            return velocity.dofCount == 0 ? 0.0 : velocity[0];
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

        private void UpdateOdometry(float deltaTime, double leftOmega, double rightOmega)
        {
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
