import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time
from tf2_ros import TransformListener, Buffer, LookupException, ExtrapolationException
import time
import math
from std_srvs.srv import Trigger
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /servo_node/start_servo service...")
        self.req = Trigger.Request()
        
        if self.start_servo():
            time.sleep(1)
            self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
            self.timer = self.create_timer(0.008, self.publish_twist)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            
            # Retry until the initial pose of the object is obtained
            self.initial_object_transform = self.get_initial_object_transform()
        else:
            self.get_logger().error("Failed to start servoing")

    def start_servo(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().success

    def get_initial_object_transform(self):
        # Keep retrying to get the initial transform with a timeout
        while rclpy.ok():
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    'base_link',  # Reference frame
                    'obj_5',    # Target frame
                    rclpy.time.Time()  # Get the latest transform
                )
                self.get_logger().info("Successfully obtained initial object transform.")
                return transform
            except Exception as e:
                self.get_logger().error(f'Could not transform: {str(e)}')

    def publish_twist(self):
        try:
            object_transform = self.initial_object_transform
            effector_transform: TransformStamped = self.tf_buffer.lookup_transform('base_link', 'wrist_3_link', rclpy.time.Time())
        except LookupException as e:
            self.get_logger().warn("Transform not available, skipping this cycle.")
            return

        obj_orientation = object_transform.transform.rotation
        effector_orientation = effector_transform.transform.rotation

        obj_euler = euler_from_quaternion([obj_orientation.x, obj_orientation.y, obj_orientation.z, obj_orientation.w])
        effector_euler = euler_from_quaternion([effector_orientation.x, effector_orientation.y, effector_orientation.z, effector_orientation.w])
        delta_orientation_y = obj_euler[0] - effector_euler[0]

        if math.fabs(delta_orientation_y) > 0.01:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.angular.y = -0.4 if delta_orientation_y > 0 else 0.4
            self.publisher.publish(twist_msg)
            self.get_logger().info(f"Aligning orientation: {twist_msg.twist.angular.y}")
            return

        delta_x = object_transform.transform.translation.x - effector_transform.transform.translation.x
        delta_y = object_transform.transform.translation.y - effector_transform.transform.translation.y
        delta_z = object_transform.transform.translation.z - effector_transform.transform.translation.z
        
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        if distance > 0.01:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = 0.3 * (delta_x / distance)
            twist_msg.twist.linear.y = 0.3 * (delta_y / distance)
            twist_msg.twist.linear.z = 0.3 * (delta_z / distance)
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
