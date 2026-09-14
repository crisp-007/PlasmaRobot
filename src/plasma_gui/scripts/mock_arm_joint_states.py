#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MockArmJointStatePublisher(Node):
    def __init__(self):
        super().__init__("mock_arm_joint_state_publisher")
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        self._started_at = self.get_clock().now()
        self._timer = self.create_timer(0.05, self._publish)
        self.get_logger().info("Publishing simulated six-axis data on /joint_states")

    def _publish(self):
        elapsed = (self.get_clock().now() - self._started_at).nanoseconds / 1e9
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f"joint{i}" for i in range(1, 7)]
        message.position = [
            math.radians(12.0 * math.sin(elapsed * 0.45 + index * 0.55))
            for index in range(6)
        ]
        message.velocity = [0.0] * 6
        message.effort = [0.0] * 6
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MockArmJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
