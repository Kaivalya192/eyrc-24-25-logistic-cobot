#!/usr/bin/env python3
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from math import sin, cos
import time

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.navigator = BasicNavigator()
        self.get_logger().info("Starting navigation node")

        # Define poses
        init_pose = self.create_pose(0.0, 0.0, 0.0)
        drop_point = self.create_pose(-0.12, -2.35, 3.14)
        conveyor_1 = self.create_pose(-4.4, 2.89, -1.57)
        conveyor_2 = self.create_pose(2.32, 2.55, -1.57)

        self.navigator.setInitialPose(init_pose)
        self.navigator.waitUntilNav2Active()

        if self.navigate_to_pose(drop_point):
            time.sleep(2)
            # #recieve the package call the service 
            # if self.navigate_to_pose(conveyor_1):
            #     time.sleep(2)
            #     if self.navigate_to_pose(drop_point):
            #         time.sleep(2)
            #         if self.navigate_to_pose(conveyor_2):
            #             time.sleep(2)

        self.check_result()

    def create_pose(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = sin(theta / 2.0)
        pose.pose.orientation.w = cos(theta / 2.0)
        return pose

    def navigate_to_pose(self, goal_pose, max_duration=600):
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            total_time_in_seconds = feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9
            if total_time_in_seconds > max_duration:
                self.navigator.cancelTask()
                self.get_logger().info("Navigation task canceled due to timeout.")
                return False
        return True

    def check_result(self):
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('All goals succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()
    exit()

if __name__ == '__main__':
    main()
