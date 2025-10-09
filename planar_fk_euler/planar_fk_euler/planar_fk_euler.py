#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D

class PlanarFKEuler(Node):
    def __init__(self):
        super().__init__('planar_fk_euler')

        # 参数化的连杆长度，可在 launch 或命令行中覆盖
        self.declare_parameter('link1', 1.0)
        self.declare_parameter('link2', 1.0)

        # 订阅 joint_states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10)

        # 发布 ee_pose2d
        self.publisher = self.create_publisher(Pose2D, 'ee_pose2d', 10)

        self.get_logger().info('✅ planar_fk_euler node started')

    def joint_callback(self, msg: JointState):
        try:
            # 假设前两个关节为 revolute 关节
            theta1 = msg.position[0]
            theta2 = msg.position[1]
        except IndexError:
            self.get_logger().warn('JointState does not contain enough joints!')
            return

        # 读取参数
        L1 = float(self.get_parameter('link1').value)
        L2 = float(self.get_parameter('link2').value)

        # 计算末端位姿
        x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
        y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
        yaw = theta1 + theta2

        pose = Pose2D()
        pose.x = x
        pose.y = y
        pose.theta = yaw

        # 发布结果
        self.publisher.publish(pose)
        self.get_logger().info(
            f'ee_pose2d → x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PlanarFKEuler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
