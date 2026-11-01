#!/usr/bin/env python3
"""Run inside the Cub_ROS container with its ROS environment sourced."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--require-both", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = Node("m5dial_connection_test")
    samples = []
    transitions = []

    def receive(message):
        if not samples or samples[-1][1] != message.data:
            transitions.append(message.data)
            print(f"button={message.data}", flush=True)
        samples.append((time.monotonic(), message.data))

    subscription = node.create_subscription(
        Bool, "/dial/button", receive,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.VOLATILE))
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        period = samples[-1][0] - samples[0][0] if len(samples) > 1 else 0
        result = {
            "topic": "/dial/button", "type": "std_msgs/msg/Bool",
            "received": len(samples), "transitions": transitions,
            "rate_hz": round((len(samples) - 1) / period, 2) if period else 0,
            "publishers": node.count_publishers("/dial/button"),
        }
        print(json.dumps(result), flush=True)
        if len(samples) < 10:
            raise SystemExit("FAIL: fewer than 10 live samples received")
        if result["publishers"] != 1:
            raise SystemExit("FAIL: expected exactly one publisher (check stale Agent sessions)")
        if args.require_both and set(transitions) != {False, True}:
            raise SystemExit("FAIL: button press and release were not both observed")
        print("PASS", flush=True)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
