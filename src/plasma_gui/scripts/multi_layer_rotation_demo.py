#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String

from rm_ros_interfaces.msg import Movej, Moveloffset


class MultiLayerRotationDemo(Node):
    def __init__(self):
        super().__init__('multi_layer_rotation_demo')
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.joints = None
        self.joint_sequence = 0
        self.movej_result = None
        self.movel_result = None
        self.tool_frame = None

        self.movej_pub = self.create_publisher(
            Movej, '/rm_driver/movej_cmd', reliable)
        self.movel_pub = self.create_publisher(
            Moveloffset, '/rm_driver/movel_offset_cmd', reliable)
        self.stop_pub = self.create_publisher(
            Empty, '/rm_driver/move_stop_cmd', reliable)
        self.tool_frame_pub = self.create_publisher(
            Empty, '/rm_driver/get_current_tool_frame_cmd', reliable)

        self.create_subscription(
            JointState, '/joint_states', self._on_joints, best_effort)
        self.create_subscription(
            Bool, '/rm_driver/movej_result', self._on_movej_result, reliable)
        self.create_subscription(
            Bool, '/rm_driver/movel_offset_result', self._on_movel_result,
            reliable)
        self.create_subscription(
            String, '/rm_driver/get_current_tool_frame_result',
            self._on_tool_frame, reliable)

    def _on_joints(self, message):
        if len(message.position) >= 6:
            self.joints = list(message.position[:6])
            self.joint_sequence += 1

    def _on_movej_result(self, message):
        self.movej_result = bool(message.data)

    def _on_movel_result(self, message):
        self.movel_result = bool(message.data)

    def _on_tool_frame(self, message):
        self.tool_frame = message.data.strip()

    def wait_for_ready(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.joints is not None
                    and self.movej_pub.get_subscription_count() > 0
                    and self.movel_pub.get_subscription_count() > 0):
                return True
        return False

    def query_tool_frame(self, timeout=3.0):
        self.tool_frame = None
        self.tool_frame_pub.publish(Empty())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tool_frame:
                return self.tool_frame
        return None

    def stop(self):
        self.stop_pub.publish(Empty())

    def move_joint(self, target, label, timeout=90.0):
        message = Movej()
        message.joint = target
        message.speed = 5
        message.block = True
        message.trajectory_connect = 0
        message.dof = 6
        self.movej_result = None
        print(f'执行：{label}，目标 J6={math.degrees(target[5]):.1f} deg')
        self.movej_pub.publish(message)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.movej_result is not None:
                if self.movej_result:
                    print(f'完成：{label}')
                    return True
                print(f'失败：{label}，已发送停止命令')
                self.stop()
                return False

        print(f'超时：{label}，已发送停止命令')
        self.stop()
        return False

    def move_tool_z(self, distance_m, label, timeout=90.0):
        message = Moveloffset()
        message.pose.position.z = distance_m
        message.pose.orientation.w = 1.0
        message.speed = 5
        message.r = 0
        message.trajectory_connect = False
        message.frame_type = True
        message.block = True

        start_sequence = self.joint_sequence
        self.movel_result = None
        print(f'执行：{label}，工具 +Z={distance_m * 1000.0:.1f} mm')
        self.movel_pub.publish(message)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.movel_result is not None:
                if not self.movel_result:
                    print(f'失败：{label}，已发送停止命令')
                    self.stop()
                    return False
                break
        else:
            print(f'超时：{label}，已发送停止命令')
            self.stop()
            return False

        state_deadline = time.monotonic() + 2.5
        while time.monotonic() < state_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_sequence > start_sequence:
                print(f'完成：{label}')
                return True

        print(f'失败：{label}后没有新的关节状态，已发送停止命令')
        self.stop()
        return False

    def rotate_layer(self, layer_number, sweep_rad):
        baseline = list(self.joints)
        left = list(baseline)
        right = list(baseline)
        left[5] += sweep_rad
        right[5] -= sweep_rad

        if max(abs(math.degrees(left[5])), abs(math.degrees(right[5]))) > 360.0:
            print(f'第 {layer_number} 层 J6 目标超过 +/-360 deg，拒绝执行')
            return False

        print(f'第 {layer_number} 层入口 J6={math.degrees(baseline[5]):.1f} deg')
        commands = (
            (left, f'第 {layer_number} 层左转半圈'),
            (baseline, f'第 {layer_number} 层左转回零'),
            (right, f'第 {layer_number} 层右转半圈'),
            (baseline, f'第 {layer_number} 层右转回零'),
        )
        return all(self.move_joint(target, label) for target, label in commands)


def main():
    parser = argparse.ArgumentParser(
        description='ECO65 工具 +Z 分层步进及 J6 左右半圈干运行')
    parser.add_argument('--execute', action='store_true',
                        help='实际下发机械臂运动')
    parser.add_argument('--additional-layers', type=int, default=2,
                        help='从当前位置向前新增的演示层数')
    parser.add_argument('--spacing-mm', type=float, default=5.0,
                        help='每个新增层沿工具 +Z 的步进距离')
    parser.add_argument('--sweep-deg', type=float, default=180.0)
    args = parser.parse_args()

    if not 1 <= args.additional_layers <= 5:
        parser.error('--additional-layers 必须在 1 到 5 之间')
    if not 0.1 <= args.spacing_mm <= 10.0:
        parser.error('--spacing-mm 必须在 0.1 到 10.0 之间')
    if args.additional_layers * args.spacing_mm > 25.0:
        parser.error('累计步进不得超过当前六层规划的 25 mm')
    if not 0.0 < args.sweep_deg <= 180.0:
        parser.error('--sweep-deg 必须在 0 到 180 之间')

    rclpy.init()
    node = MultiLayerRotationDemo()
    try:
        if not node.wait_for_ready():
            print('机械臂驱动、MoveJ、MoveL 或 /joint_states 未就绪')
            return 2

        tool_frame = node.query_tool_frame()
        if tool_frame != 'Arm_Tip':
            print(f'当前工具系为 {tool_frame or "未知"}，要求 Arm_Tip，拒绝执行')
            return 2

        total_mm = args.additional_layers * args.spacing_mm
        print(f'当前工具系：{tool_frame}')
        print(f'当前 J6={math.degrees(node.joints[5]):.1f} deg')
        print(f'新增层数={args.additional_layers}，层间距={args.spacing_mm:.1f} mm，'
              f'累计工具 +Z={total_mm:.1f} mm')
        print('每个新增层：前进 -> 左转半圈 -> 回零 -> 右转半圈 -> 回零')
        print('速度=5%，真实等离子输出=关闭；结束后停留在最后一层')

        if not args.execute:
            print('仅预览；确认现场安全后增加 --execute')
            return 0

        print('5 秒后开始运动；需要取消请立即按 Ctrl+C')
        for remaining in range(5, 0, -1):
            print(f'{remaining}...')
            time.sleep(1)

        sweep_rad = math.radians(args.sweep_deg)
        for added_index in range(args.additional_layers):
            layer_number = added_index + 2
            if not node.move_tool_z(
                    args.spacing_mm / 1000.0,
                    f'移动到第 {layer_number} 层'):
                return 1
            if not node.rotate_layer(layer_number, sweep_rad):
                return 1

        print(f'新增 {args.additional_layers} 层演示完成，累计前进 {total_mm:.1f} mm；'
              'J6 已回到末层入口角度')
        return 0
    except KeyboardInterrupt:
        node.stop()
        print('\n已中断并发送停止命令')
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
