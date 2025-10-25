#!/usr/bin/env python3
"""Convert EKF pose stream into a nav_msgs/Path sequence."""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile


class EkfPosePathPublisher(Node):
	def __init__(self) -> None:
		super().__init__('ekf_pose_path_publisher')

		self.pose_topic: str = self.declare_parameter('pose_topic', '/test/ekf_pose').value
		self.path_topic: str = self.declare_parameter('path_topic', '/test/ekf_path').value
		self.max_path_length: int = int(self.declare_parameter('max_path_length', 100000).value)
		self.override_frame_id: str = self.declare_parameter('override_frame_id', '').value

		pose_qos = QoSProfile(depth=50)
		path_qos = QoSProfile(depth=10)

		self._path_msg = Path()
		self._last_frame_id: Optional[str] = None
		self._warned_frame_jump = False
		self._last_header_time = 0

		self._path_publisher = self.create_publisher(Path, self.path_topic, path_qos)
		self._pose_subscription = self.create_subscription(
			PoseStamped,
			self.pose_topic,
			self._on_pose,
			pose_qos,
		)

		self.get_logger().info(
			f"Streaming poses from {self.pose_topic} to {self.path_topic} with max length {self.max_path_length}"
		)

	def _on_pose(self, msg: PoseStamped) -> None:
		frame_id = self.override_frame_id or msg.header.frame_id

		if self._last_frame_id is not None and frame_id != self._last_frame_id and not self._warned_frame_jump:
			self.get_logger().warn(
				f"Frame ID changed from {self._last_frame_id} to {frame_id}; using latest for path header"
			)
			self._warned_frame_jump = True
		if self._last_header_time  > msg.header.stamp.sec:
			return
		self._last_header_time = msg.header.stamp.sec
		self._last_frame_id = frame_id

		pose = PoseStamped()
		pose.header = msg.header
		pose.header.frame_id = frame_id
		pose.pose = msg.pose

		self._path_msg.header.stamp = msg.header.stamp
		self._path_msg.header.frame_id = frame_id
		self._path_msg.poses.append(pose)

		# Prevent the list from growing unbounded when a maximum length is provided.
		if self.max_path_length > 0 and len(self._path_msg.poses) > self.max_path_length:
			self._path_msg.poses.pop(0)

		self._path_publisher.publish(self._path_msg)


def main() -> None:
	rclpy.init()
	node = EkfPosePathPublisher()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
