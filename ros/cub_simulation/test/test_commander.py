"""Run the real C++ commander and check control arbitration and watchdog behavior."""
import os
import subprocess
import signal
import time
import pytest
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


@pytest.fixture
def commander(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '172')
    rclpy.init()
    node = rclpy.create_node('commander_control_test')
    received = []
    subscription = node.create_subscription(Twist, '/test/cmd', received.append, 10)
    joy_pub = node.create_publisher(Joy, '/test/joy', 10)
    nav_pub = node.create_publisher(Twist, '/test/nav', 10)
    process = subprocess.Popen([
        'ros2', 'run', 'cub_commander', 'cub_commander_node', '--ros-args',
        '-r', '/joy:=/test/joy', '-r', '/cmd_vel:=/test/nav', '-r', '/cmd_vel_atom:=/test/cmd'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        deadline = time.monotonic() + 10
        while not joy_pub.get_subscription_count() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert joy_pub.get_subscription_count(), 'commander did not start'
        yield node, process, received, joy_pub, nav_pub
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


def joy(forward=0.0, left=0.0, manual=True):
    message = Joy()
    message.axes = [left, forward, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    message.buttons = [0] * 14
    message.buttons[4] = int(manual)
    return message


def spin(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def publish_for(node, publisher, message, seconds=0.2):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if isinstance(message, Joy):
            message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        spin(node, 0.04)


def stopped(message):
    return message.linear.x == 0.0 and message.angular.z == 0.0


def test_manual_stop_timeout_and_autonomous_arbitration(commander):
    node, process, received, joy_pub, nav_pub = commander
    publish_for(node, joy_pub, joy(1.0, 1.0))
    assert received and received[-1].linear.x == pytest.approx(1.2)
    assert received[-1].angular.z == pytest.approx(1.0471975512)
    publish_for(node, joy_pub, joy())
    assert stopped(received[-1]), 'neutral manual input must stop'
    nav = Twist()
    nav.linear.x = 0.4
    publish_for(node, nav_pub, nav)
    assert stopped(received[-1]), 'holding manual zero overrides autonomous input'
    publish_for(node, joy_pub, joy(manual=False))
    publish_for(node, nav_pub, nav)
    assert received[-1].linear.x == pytest.approx(0.4), 'manual release accepts the next autonomous command'
    publish_for(node, joy_pub, joy(1.0))
    publish_for(node, nav_pub, nav, seconds=1.3)
    assert received[-1].linear.x == pytest.approx(0.4), 'original 1-second Joy timeout returns control to autonomy'
    spin(node, 3.3)
    assert stopped(received[-1]), 'original 3-second autonomous command timeout must stop'
    assert process.poll() is None
