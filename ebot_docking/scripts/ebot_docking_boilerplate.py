#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw
import math

class MyRobotDockingController(Node):
    def __init__(self):
        super().__init__('my_robot_docking_controller')

        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Service
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Docking state flags and parameters
        self.is_docking = False
        self.linear_dock = False
        self.orientation_dock = False
        self.target_distance = 0.0
        self.target_orientation = 0.0
        self.distance_tolerance = 0.05  # Adjust as needed
        self.orientation_tolerance = math.radians(5)  # 5 degrees tolerance

        # Sensor data placeholders
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, yaw (from odometry)
        self.imu_yaw = 0.0  # Orientation from IMU
        self.usrleft_value = float('inf')
        self.usrright_value = float('inf')

        # P-controller gains
        self.linear_kp = 0.5
        self.angular_kp = 1.0

        # Timer for control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback for odometry
    def odometry_callback(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        _, _, self.robot_pose[2] = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

    # Callback for IMU
    def imu_callback(self, msg):
        quaternion = msg.orientation
        _, _, self.imu_yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

    # Callback for left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    # Callback for right ultrasonic sensor
    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range

    # Main control loop
    def controller_loop(self):
        if not self.is_docking:
            return

        # Calculate error and control signals for linear docking
        if self.linear_dock:
            avg_distance = (self.usrleft_value + self.usrright_value) / 2
            distance_error = self.target_distance - avg_distance

            if abs(distance_error) > self.distance_tolerance:
                linear_speed = self.linear_kp * distance_error
            else:
                linear_speed = 0.0
                self.linear_dock = False  # Linear docking achieved

        # Calculate error and control signals for orientation docking
        if self.orientation_dock:
            orientation_error = self.target_orientation - self.imu_yaw
            # Normalize angle to range [-pi, pi]
            orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))

            if abs(orientation_error) > self.orientation_tolerance:
                angular_speed = self.angular_kp * orientation_error
            else:
                angular_speed = 0.0
                self.orientation_dock = False  # Orientation docking achieved

        # Publish velocity commands
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed if self.linear_dock else 0.0
        cmd_vel.angular.z = angular_speed if self.orientation_dock else 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        # Stop docking if both linear and orientation docking are complete
        if not self.linear_dock and not self.orientation_dock:
            self.is_docking = False
            self.get_logger().info("Docking complete.")

    # Service callback
    def dock_control_callback(self, request, response):
        self.is_docking = True
        self.linear_dock = request.linear_dock
        self.orientation_dock = request.orientation_dock
        self.target_distance = request.distance
        self.target_orientation = request.orientation

        self.get_logger().info("Docking started with parameters - Linear Dock: {}, Orientation Dock: {}, Target Distance: {:.2f}, Target Orientation: {:.2f}".format(
            self.linear_dock, self.orientation_dock, self.target_distance, math.degrees(self.target_orientation)))

        response.success = True
        response.message = "Docking control initiated"
        return response

def main(args=None):
    rclpy.init(args=args)
    my_robot_docking_controller = MyRobotDockingController()
    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)
    executor.spin()
    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
