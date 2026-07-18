#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage


class CubSimulationSmoke(Node):
    def __init__(self):
        super().__init__('cub_simulation_smoke')
        self.declare_parameter('timeout', 20.0)
        self.deadline = time.monotonic() + float(self.get_parameter('timeout').value)
        self.command_start = None
        self.scan_count = 0
        self.odom_count = 0
        self.joint_state_count = 0
        self.tf_count = 0
        self.start_x = None
        self.travel = 0.0
        self.failure = None
        self.success = False
        self.command = self.create_publisher(Twist, '/cmd_vel_atom', 10)
        self.create_subscription(Odometry, '/odom', self.on_odometry, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 10)
        self.create_timer(0.05, self.tick)

    def on_odometry(self, message):
        self.odom_count += 1
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
            self.failure = 'unexpected odometry frame IDs'
            return
        if self.start_x is None:
            self.start_x = message.pose.pose.position.x
        self.travel = abs(message.pose.pose.position.x - self.start_x)

    def on_scan(self, message):
        self.scan_count += 1
        checks = [
            (message.header.frame_id == 'SLC1_link', 'unexpected LaserScan frame ID'),
            (len(message.ranges) == 500, f'expected 500 C1 samples, got {len(message.ranges)}'),
            (math.isclose(message.angle_increment, math.radians(0.72), rel_tol=1e-4), 'unexpected C1 angle increment'),
            (math.isclose(message.scan_time, 0.1, rel_tol=1e-4), 'unexpected C1 scan time'),
            (math.isclose(message.time_increment, 1.0 / 5000.0, rel_tol=1e-4), 'unexpected C1 sample period'),
            (math.isclose(message.range_min, 0.05, rel_tol=1e-4), 'unexpected C1 minimum range'),
            (math.isclose(message.range_max, 12.0, rel_tol=1e-4), 'unexpected C1 maximum range'),
        ]
        for valid, reason in checks:
            if not valid:
                self.failure = reason
                return

    def on_tf(self, message):
        for transform in message.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                self.tf_count += 1

    def on_joint_state(self, message):
        expected = {
            'left_wheel_joint',
            'right_wheel_joint',
            'caster_swivel_joint',
            'caster_wheel_joint',
        }
        if set(message.name) != expected or len(message.position) != len(expected):
            self.failure = 'unexpected joint state names or positions'
            return
        self.joint_state_count += 1

    def tick(self):
        if self.failure:
            self.finish(False, self.failure)
            return
        now = time.monotonic()
        if self.command_start is None and self.scan_count > 0 and self.odom_count > 0 and self.tf_count > 0:
            self.command_start = now
            self.get_logger().info('Unity topics received; commanding mCub forward.')

        command = Twist()
        if self.command_start is not None and now - self.command_start < 2.0:
            command.linear.x = 0.10
        self.command.publish(command)

        if (
            self.scan_count >= 10
            and self.odom_count >= 20
            and self.joint_state_count >= 20
            and self.tf_count >= 20
            and self.travel >= 0.02
        ):
            self.finish(
                True,
                f'passed: scans={self.scan_count}, odom={self.odom_count}, '
                f'joint_states={self.joint_state_count}, tf={self.tf_count}, '
                f'travel={self.travel:.3f} m',
            )
        elif now >= self.deadline:
            self.finish(
                False,
                f'timeout: scans={self.scan_count}, odom={self.odom_count}, '
                f'joint_states={self.joint_state_count}, tf={self.tf_count}, '
                f'travel={self.travel:.3f} m',
            )

    def finish(self, success, message):
        if self.success or self.failure == '__finished__':
            return
        self.success = success
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        self.failure = '__finished__'
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CubSimulationSmoke()
    try:
        rclpy.spin(node)
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main(sys.argv)
