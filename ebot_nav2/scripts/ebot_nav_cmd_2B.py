#!/usr/bin/env python3
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from payload_service.srv import PayloadSW
from ebot_docking.srv import DockSw
import rclpy
from rclpy.node import Node
from math import sin, cos

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Starting navigation node")

        # Box service client for loading/unloading actions
        self.box_service = self.create_client(PayloadSW, 'payload_sw')
        self.box_service.wait_for_service()

        # Docking service client
        self.dock_service = self.create_client(DockSw, 'dock_control')
        self.dock_service.wait_for_service()

        drop_point_1 = self.create_pose(0.43, -2.43, 1.57)
        drop_point_2 = self.create_pose(0.43, -2.43, -1.57)
        conveyor_2 = self.create_pose(2.32, 2.55, -1.57)
        conveyor_1 = self.create_pose(-4.5, 2.89, -1.57)

        if self.navigate_to_pose(drop_point_1):
            time.sleep(1)
            self.box_action(True, False)
            time.sleep(1)
            if self.navigate_to_pose(conveyor_2):
                time.sleep(1)
                self.dock_action(True, False, 0.0, -1.57)
                time.sleep(7)
                self.box_action(False, True)
                time.sleep(1)
                if self.navigate_to_pose(drop_point_2):
                    time.sleep(1)
                    self.box_action(True, False)
                    time.sleep(1)
                    if self.navigate_to_pose(conveyor_1):
                        time.sleep(1)
                        self.dock_action(True, True, 0.0, -1.57)
                        time.sleep(7)
                        self.box_action(False, True)

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
            if feedback:
                total_time_in_seconds = feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9
                if total_time_in_seconds > max_duration:
                    self.navigator.cancelTask()
                    self.get_logger().info("Navigation task canceled due to timeout.")
                    return False
        return True

    def box_action(self, receive, drop):
        request = PayloadSW.Request()
        request.receive = receive
        request.drop = drop
        future = self.box_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response and response.success:
            self.get_logger().info(f"Box action succeeded: {response.message}")
        else:
            self.get_logger().info(f"Box action failed: {response.message if response else 'No response'}")

    def dock_action(self, linear_dock, orientation_dock, distance, orientation):
        """Calls the docking service with specified docking parameters."""
        request = DockSw.Request()
        request.linear_dock = linear_dock
        request.orientation_dock = orientation_dock
        request.distance = distance
        request.orientation = orientation

        future = self.dock_service.call_async(request)
        self.get_logger().info(f"Initiating docking: Distance {distance}, Orientation {orientation}")
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response and response.success:
            self.get_logger().info(f"Docking succeeded: {response.message}")
        else:
            self.get_logger().info(f"Docking failed: {response.message if response else 'No response'}")

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

if __name__ == '__main__':
    main()
