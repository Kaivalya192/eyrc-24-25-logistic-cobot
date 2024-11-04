import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import TransformListener, Buffer, LookupException, ExtrapolationException
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion
import math


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Create client for starting the servo
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        self.wait_for_service()

        self.req = Trigger.Request()
        
        if self.start_servo():
            self.initialize_publisher_and_timer()
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().error("Failed to start servoing")

    def wait_for_service(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /servo_node/start_servo service...")

    def start_servo(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().success

    def initialize_publisher_and_timer(self):
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.create_timer(0.008, self.publish_twist)

    def publish_twist(self):
        try:
            object_transform = self.lookup_transform('base_link', 'obj_3')
            effector_transform = self.lookup_transform('base_link', 'wrist_3_link')
        except (LookupException, ExtrapolationException):
            self.get_logger().warn("Transform not available, skipping this cycle.")
            return

        if not self.align_orientation(object_transform, effector_transform):
            self.move_towards_target(object_transform, effector_transform)

    def lookup_transform(self, from_frame, to_frame):
        return self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())

    def align_orientation(self, object_transform, effector_transform):
        obj_euler = euler_from_quaternion([
            object_transform.transform.rotation.x,
            object_transform.transform.rotation.y,
            object_transform.transform.rotation.z,
            object_transform.transform.rotation.w
        ])
        
        effector_euler = euler_from_quaternion([
            effector_transform.transform.rotation.x,
            effector_transform.transform.rotation.y,
            effector_transform.transform.rotation.z,
            effector_transform.transform.rotation.w
        ])
        
        delta_orientation_y = obj_euler[0] - effector_euler[0]

        if math.fabs(delta_orientation_y) > 0.01:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.angular.y = -0.4 if delta_orientation_y > 0 else 0.4
            self.publisher.publish(twist_msg)
            self.get_logger().info(f"Aligning orientation: {twist_msg.twist.angular.y}")
            return True
        
        return False

    def move_towards_target(self, object_transform, effector_transform):
        delta_x = object_transform.transform.translation.x - effector_transform.transform.translation.x
        delta_y = object_transform.transform.translation.y - effector_transform.transform.translation.y
        delta_z = object_transform.transform.translation.z - effector_transform.transform.translation.z
        
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        
        if distance > 0.01:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            
            scale_factor = 0.3 / distance
            twist_msg.twist.linear.x = scale_factor * delta_x
            twist_msg.twist.linear.y = scale_factor * delta_y
            twist_msg.twist.linear.z = scale_factor * delta_z
            
            self.publisher.publish(twist_msg)
            self.get_logger().info(f"Moving towards target: linear x={twist_msg.twist.linear.x}, y={twist_msg.twist.linear.y}, z={twist_msg.twist.linear.z}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()