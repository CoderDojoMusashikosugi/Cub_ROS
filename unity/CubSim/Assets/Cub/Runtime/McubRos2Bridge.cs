using System;
using UnityEngine;

#if CUB_ROS2
using ROS2;
#endif

namespace CubSim
{
    public sealed class McubRos2Bridge : MonoBehaviour
    {
        private McubRobot robot;
        private IC1ScanSource lidar;

#if CUB_ROS2
        private ROS2UnityComponent ros2Unity;
        private ROS2Node node;
        private ISubscription<geometry_msgs.msg.Twist> commandSubscription;
        private IPublisher<nav_msgs.msg.Odometry> odometryPublisher;
        private IPublisher<sensor_msgs.msg.LaserScan> scanPublisher;
        private IPublisher<sensor_msgs.msg.JointState> jointStatePublisher;
        private IPublisher<rosgraph_msgs.msg.Clock> clockPublisher;
        private IPublisher<tf2_msgs.msg.TFMessage> transformPublisher;
        private IPublisher<sensor_msgs.msg.Joy> joyPublisher;
        private IPublisher<std_msgs.msg.String> controllerTypePublisher;
        private readonly object commandLock = new object();
        private float commandLinear;
        private float commandAngular;
        private bool hasCommand;
        private float nextOdometryPublishTime;
        private float nextJoyPublishTime;
        private float nextControllerTypePublishTime;
        private float joyReadyTime;
        private string announcedControllerType = string.Empty;
        private int announcedControllerDeviceId = -1;
        private bool hadSupportedController;
        private string lastControllerDiagnostic = string.Empty;
        private bool shuttingDown;
#endif

        public string ControllerStatus { get; private set; } = "Controller: waiting";

        public bool IsRosReady
        {
            get
            {
#if CUB_ROS2
                return !shuttingDown && node != null;
#else
                return false;
#endif
            }
        }

        public void Initialize(McubRobot mcubRobot, IC1ScanSource c1Lidar)
        {
            robot = mcubRobot;
            lidar = c1Lidar;
#if CUB_ROS2
            ros2Unity = gameObject.AddComponent<ROS2UnityComponent>();
            lidar.ScanReady += PublishScan;
#else
            Debug.LogWarning("CubSim was built without CUB_ROS2. Physics and LiDAR run locally, but ROS topics are disabled.");
#endif
        }

#if CUB_ROS2
        private void Update()
        {
            if (shuttingDown) return;
            if (ros2Unity == null || !ros2Unity.Ok()) return;
            if (node == null) CreateRosEntities();
        }

        private void FixedUpdate()
        {
            if (shuttingDown) return;
            if (node == null) return;
            PublishGamepad();
            lock (commandLock)
            {
                if (hasCommand)
                {
                    robot.SetVelocityCommand(commandLinear, commandAngular);
                    hasCommand = false;
                }
            }

            if (Time.time + 1e-6f < nextOdometryPublishTime) return;
            nextOdometryPublishTime = Time.time + 0.02f;
            PublishOdometry();
            PublishClock();
        }

        private void CreateRosEntities()
        {
            node = ros2Unity.CreateNode("mcub_unity_sim");
            commandSubscription = node.CreateSubscription<geometry_msgs.msg.Twist>("/cmd_vel_atom", message =>
            {
                lock (commandLock)
                {
                    commandLinear = (float)message.Linear.X;
                    commandAngular = (float)message.Angular.Z;
                    hasCommand = true;
                }
            });
            odometryPublisher = node.CreatePublisher<nav_msgs.msg.Odometry>("/odom");
            scanPublisher = node.CreateSensorPublisher<sensor_msgs.msg.LaserScan>("/scan");
            jointStatePublisher = node.CreatePublisher<sensor_msgs.msg.JointState>("/joint_states");
            clockPublisher = node.CreatePublisher<rosgraph_msgs.msg.Clock>("/clock");
            transformPublisher = node.CreatePublisher<tf2_msgs.msg.TFMessage>("/tf");
            joyPublisher = node.CreatePublisher<sensor_msgs.msg.Joy>("/joy");
            controllerTypePublisher = node.CreatePublisher<std_msgs.msg.String>("/joy/controller_type");
            Debug.Log("ROS 2 ready: subscribe /cmd_vel_atom; publish /joy, /joy/controller_type, /odom, /scan, /joint_states, /tf and /clock.");
        }

        private void PublishGamepad()
        {
            var now = Time.unscaledTime;
            if (now + 1e-6f < nextJoyPublishTime) return;
            nextJoyPublishTime = now + 0.02f;

            if (!UnityGamepadJoySource.TryRead(out var sample))
            {
                if (hadSupportedController)
                {
                    PublishNeutralJoy(announcedControllerType);
                    Debug.LogWarning("Supported gamepad disconnected; published a neutral /joy sample.");
                }
                hadSupportedController = false;
                ControllerStatus = "Controller: no supported device";
                var diagnostic = UnityGamepadJoySource.DescribeDevices();
                if (diagnostic != lastControllerDiagnostic)
                {
                    lastControllerDiagnostic = diagnostic;
                    Debug.LogWarning($"No supported gamepad detected. {diagnostic}");
                }
                return;
            }

            hadSupportedController = true;
            if (sample.ControllerType != announcedControllerType ||
                sample.DeviceId != announcedControllerDeviceId)
            {
                announcedControllerType = sample.ControllerType;
                announcedControllerDeviceId = sample.DeviceId;
                joyReadyTime = now + 0.1f;
                nextControllerTypePublishTime = 0f;
                ControllerStatus = $"Controller: {sample.Description} ({sample.ControllerType})";
                Debug.Log($"Gamepad detected: {sample.Description}; ROS type={sample.ControllerType}");
            }

            if (now + 1e-6f >= nextControllerTypePublishTime)
            {
                controllerTypePublisher.Publish(new std_msgs.msg.String { Data = announcedControllerType });
                nextControllerTypePublishTime = now + 1f;
            }
            if (now < joyReadyTime) return;

            var message = new sensor_msgs.msg.Joy
            {
                Axes = sample.Axes,
                Buttons = sample.Buttons
            };
            node.clock.UpdateROSClockTime(message.Header.Stamp);
            joyPublisher.Publish(message);
        }

        private void PublishNeutralJoy(string controllerType)
        {
            var message = new sensor_msgs.msg.Joy
            {
                Axes = new float[8],
                Buttons = new int[controllerType == "xbox" ? 15 : 14]
            };
            node.clock.UpdateROSClockTime(message.Header.Stamp);
            joyPublisher.Publish(message);
        }

        private void PublishOdometry()
        {
            var sample = robot.Odometry;
            var message = new nav_msgs.msg.Odometry();
            FillStamp(message.Header.Stamp, sample.Time);
            message.Header.Frame_id = "odom";
            message.Child_frame_id = "base_link";
            message.Pose.Pose.Position.X = sample.X;
            message.Pose.Pose.Position.Y = sample.Y;
            message.Pose.Pose.Position.Z = 0.0;
            message.Pose.Pose.Orientation.Z = Math.Sin(sample.Yaw * 0.5);
            message.Pose.Pose.Orientation.W = Math.Cos(sample.Yaw * 0.5);
            message.Twist.Twist.Linear.X = sample.LinearVelocity;
            message.Twist.Twist.Angular.Z = sample.AngularVelocity;
            message.Pose.Covariance[0] = 0.0025;
            message.Pose.Covariance[7] = 0.0025;
            message.Pose.Covariance[35] = 0.01;
            message.Twist.Covariance[0] = 0.01;
            message.Twist.Covariance[35] = 0.02;
            odometryPublisher.Publish(message);
            PublishJointState(sample);
            PublishTransform(sample);
        }

        private void PublishJointState(WheelOdometrySample sample)
        {
            var message = new sensor_msgs.msg.JointState
            {
                Name = new[]
                {
                    "left_wheel_joint",
                    "right_wheel_joint",
                    "caster_swivel_joint",
                    "caster_wheel_joint"
                },
                Position = new[]
                {
                    sample.LeftWheelAngle,
                    sample.RightWheelAngle,
                    0.0,
                    0.0
                },
                Velocity = Array.Empty<double>(),
                Effort = Array.Empty<double>()
            };
            FillStamp(message.Header.Stamp, sample.Time);
            jointStatePublisher.Publish(message);
        }

        private void PublishTransform(WheelOdometrySample sample)
        {
            var transform = new geometry_msgs.msg.TransformStamped();
            FillStamp(transform.Header.Stamp, sample.Time);
            transform.Header.Frame_id = "odom";
            transform.Child_frame_id = "base_link";
            transform.Transform.Translation.X = sample.X;
            transform.Transform.Translation.Y = sample.Y;
            transform.Transform.Translation.Z = 0.0;
            transform.Transform.Rotation.Z = Math.Sin(sample.Yaw * 0.5);
            transform.Transform.Rotation.W = Math.Cos(sample.Yaw * 0.5);
            transformPublisher.Publish(new tf2_msgs.msg.TFMessage
            {
                Transforms = new[] { transform }
            });
        }

        private void PublishScan(C1Scan scan)
        {
            if (shuttingDown || scanPublisher == null) return;
            var message = new sensor_msgs.msg.LaserScan
            {
                Angle_min = scan.AngleMin,
                Angle_max = scan.AngleMax,
                Angle_increment = scan.AngleIncrement,
                Time_increment = scan.TimeIncrement,
                Scan_time = scan.ScanTime,
                Range_min = scan.RangeMin,
                Range_max = scan.RangeMax,
                Ranges = scan.Ranges,
                Intensities = scan.Intensities
            };
            FillStamp(message.Header.Stamp, scan.Time);
            message.Header.Frame_id = "SLC1_link";
            scanPublisher.Publish(message);
        }

        private void PublishClock()
        {
            var message = new rosgraph_msgs.msg.Clock();
            FillStamp(message.Clock_, Time.timeAsDouble);
            clockPublisher.Publish(message);
        }

        private static void FillStamp(builtin_interfaces.msg.Time stamp, double time)
        {
            var seconds = Math.Floor(time);
            stamp.Sec = (int)seconds;
            stamp.Nanosec = (uint)((time - seconds) * 1_000_000_000.0);
        }

        private void OnDisable()
        {
            ShutdownRos();
        }

        private void OnDestroy()
        {
            ShutdownRos();
        }

        private void ShutdownRos()
        {
            if (shuttingDown) return;
            shuttingDown = true;
            if (lidar != null) lidar.ScanReady -= PublishScan;
            if (node == null || ros2Unity == null || !ros2Unity.Ok()) return;
            if (commandSubscription != null) node.RemoveSubscription<geometry_msgs.msg.Twist>(commandSubscription);
            if (odometryPublisher != null) node.RemovePublisher<nav_msgs.msg.Odometry>(odometryPublisher);
            if (scanPublisher != null) node.RemovePublisher<sensor_msgs.msg.LaserScan>(scanPublisher);
            if (jointStatePublisher != null) node.RemovePublisher<sensor_msgs.msg.JointState>(jointStatePublisher);
            if (clockPublisher != null) node.RemovePublisher<rosgraph_msgs.msg.Clock>(clockPublisher);
            if (transformPublisher != null) node.RemovePublisher<tf2_msgs.msg.TFMessage>(transformPublisher);
            if (joyPublisher != null) node.RemovePublisher<sensor_msgs.msg.Joy>(joyPublisher);
            if (controllerTypePublisher != null) node.RemovePublisher<std_msgs.msg.String>(controllerTypePublisher);
            ros2Unity.RemoveNode(node);
            node = null;
        }
#endif
    }
}
