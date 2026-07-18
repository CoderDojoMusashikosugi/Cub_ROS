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
        private SlamtecC1Lidar lidar;

#if CUB_ROS2
        private ROS2UnityComponent ros2Unity;
        private ROS2Node node;
        private ISubscription<geometry_msgs.msg.Twist> commandSubscription;
        private IPublisher<nav_msgs.msg.Odometry> odometryPublisher;
        private IPublisher<sensor_msgs.msg.LaserScan> scanPublisher;
        private IPublisher<sensor_msgs.msg.JointState> jointStatePublisher;
        private IPublisher<rosgraph_msgs.msg.Clock> clockPublisher;
        private IPublisher<tf2_msgs.msg.TFMessage> transformPublisher;
        private readonly object commandLock = new object();
        private float commandLinear;
        private float commandAngular;
        private bool hasCommand;
        private float nextOdometryPublishTime;
#endif

        public bool IsRosReady
        {
            get
            {
#if CUB_ROS2
                return node != null;
#else
                return false;
#endif
            }
        }

        public void Initialize(McubRobot mcubRobot, SlamtecC1Lidar c1Lidar)
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
            if (ros2Unity == null || !ros2Unity.Ok()) return;
            if (node == null) CreateRosEntities();
        }

        private void FixedUpdate()
        {
            if (node == null) return;
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
            Debug.Log("ROS 2 ready: subscribe /cmd_vel_atom; publish /odom, /scan, /joint_states, /tf and /clock.");
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
            if (scanPublisher == null) return;
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

        private void OnDestroy()
        {
            if (lidar != null) lidar.ScanReady -= PublishScan;
            if (node == null || ros2Unity == null || !ros2Unity.Ok()) return;
            if (commandSubscription != null) node.RemoveSubscription<geometry_msgs.msg.Twist>(commandSubscription);
            if (odometryPublisher != null) node.RemovePublisher<nav_msgs.msg.Odometry>(odometryPublisher);
            if (scanPublisher != null) node.RemovePublisher<sensor_msgs.msg.LaserScan>(scanPublisher);
            if (jointStatePublisher != null) node.RemovePublisher<sensor_msgs.msg.JointState>(jointStatePublisher);
            if (clockPublisher != null) node.RemovePublisher<rosgraph_msgs.msg.Clock>(clockPublisher);
            if (transformPublisher != null) node.RemovePublisher<tf2_msgs.msg.TFMessage>(transformPublisher);
            ros2Unity.RemoveNode(node);
        }
#endif
    }
}
