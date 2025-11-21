#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class SetInitialPoseNode(Node):
    def __init__(self):
        super().__init__('set_initial_pose_node')

        self.navigator = BasicNavigator()

        # -------------------------
        # Task 2: Set the initial pose
        # -------------------------
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        initial_pose.pose.position.x = -2.0
        initial_pose.pose.position.y = -0.5
        initial_pose.pose.orientation.w = 1.0

        self.get_logger().info("Setting initial pose...")
        self.navigator.setInitialPose(initial_pose)
        # self.navigator.waitUntilNav2Active() # uncomment if Nav2 is not already active
        self.get_logger().info("Initial pose set!")

        # -------------------------
        # Task 3: Set the nav goal
        # -------------------------
        self.get_logger().info("Waiting for Nav2 activation...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active!")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        goal_pose.pose.position.x = 1.5
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info("Sending navigation goal...")
        self.navigator.goToPose(goal_pose)
        self.get_logger().info("Goal sent. Node exiting.")

        # -------------------------
        # Task 4: monitor the navigation status
        # -------------------------
        while not self.navigator.isTaskComplete():
            self.get_logger().info("Navigating...")
            time.sleep(1.0)  # delay for demonstration purposes

        status = self.navigator.getResult()
        self.get_logger().info(f"navigation result: {status}")

        # -------------------------
        # Task 5: Automatically cancel the navigation after a timeout period
        # -------------------------
        # timeout_sec = 10.0
        # start_time = time.time()

        # while not navigator.isTaskComplete():

        #     current_time = time.time()
        #     elapsed = current_time - start_time

        #     self.get_logger().info(f"Navigating... elapsed={elapsed:.1f}s")

        #     # 判断是否超时
        #     if elapsed > timeout_sec:
        #         self.get_logger().warn("Timeout reached! Canceling task...")
        #         self.navigator.cancelTask()
        #         break

        #     time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPoseNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
