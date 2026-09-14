#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty

from rm_ros_interfaces.msg import Movej


class SingleLayerRotationDemo(Node):
    def __init__(self):
        super().__init__('single_layer_rotation_demo')
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joints = None
        self.move_result = None
        self.move_pub = self.create_publisher(Movej, '/rm_driver/movej_cmd', reliable)
        self.stop_pub = self.create_publisher(Empty, '/rm_driver/move_stop_cmd', reliable)
        self.create_subscription(JointState, '/joint_states', self._on_joints, best_effort)
        self.create_subscription(Bool, '/rm_driver/movej_result', self._on_result, reliable)

    def _on_joints(self, message):
        if len(message.position) >= 6:
            self.joints = list(message.position[:6])

    def _on_result(self, message):
        self.move_result = bool(message.data)

    def wait_for_ready(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joints is not None and self.move_pub.get_subscription_count() > 0:
                return True
        return False

    def move_to(self, target, label, timeout=90.0):
        message = Movej()
        message.joint = target
        message.speed = 5
        message.block = True
        message.trajectory_connect = 0
        message.dof = 6
        self.move_result = None
        print(f'执行：{label}，目标 J6={math.degrees(target[5]):.1f} deg')
        self.move_pub.publish(message)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.move_result is not None:
                if self.move_result:
                    print(f'完成：{label}')
                    return True
                print(f'失败：{label}，已发送停止命令')
                self.stop_pub.publish(Empty())
                return False

        print(f'超时：{label}，已发送停止命令')
        self.stop_pub.publish(Empty())
        return False


def main():
    parser = argparse.ArgumentParser(description='ECO65 单层 J6 左右半圈干运行')
    parser.add_argument('--execute', action='store_true', help='实际下发机械臂运动')
    parser.add_argument('--sweep-deg', type=float, default=180.0)
    args = parser.parse_args()

    if not 0.0 < args.sweep_deg <= 180.0:
        parser.error('--sweep-deg 必须在 0 到 180 之间')

    rclpy.init()
    node = SingleLayerRotationDemo()
    try:
        if not node.wait_for_ready():
            print('机械臂驱动或 /joint_states 未就绪')
            return 2

        baseline = list(node.joints)
        sweep = math.radians(args.sweep_deg)
        left = list(baseline)
        right = list(baseline)
        left[5] += sweep
        right[5] -= sweep
        print(f'当前 J6={math.degrees(baseline[5]):.1f} deg')
        print(f'演示序列：{math.degrees(left[5]):.1f} -> '
              f'{math.degrees(baseline[5]):.1f} -> '
              f'{math.degrees(right[5]):.1f} -> '
              f'{math.degrees(baseline[5]):.1f} deg')

        if max(abs(math.degrees(left[5])), abs(math.degrees(right[5]))) > 360.0:
            print('J6 目标超过 +/-360 deg，拒绝执行')
            return 2
        if not args.execute:
            print('仅预览；确认现场安全后增加 --execute')
            return 0

        print('5 秒后开始运动；需要取消请立即按 Ctrl+C')
        for remaining in range(5, 0, -1):
            print(f'{remaining}...')
            time.sleep(1)

        for target, label in (
            (left, '左转半圈'),
            (baseline, '左转回零'),
            (right, '右转半圈'),
            (baseline, '右转回零'),
        ):
            if not node.move_to(target, label):
                return 1
        print('单层左右半圈演示完成，J6 已回到入口角度')
        return 0
    except KeyboardInterrupt:
        node.stop_pub.publish(Empty())
        print('\n已中断并发送停止命令')
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
