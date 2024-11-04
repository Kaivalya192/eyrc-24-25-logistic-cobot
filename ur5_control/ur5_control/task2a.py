import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time
from tf2_ros import TransformListener, Buffer
import time
import math
from std_srvs.srv import Trigger
from linkattacher_msgs.srv import AttachLink, DetachLink
from servo_msgs.srv import ServoLink
from tf_transformations import euler_from_quaternion


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Service clients for attaching, detaching, and removing boxes
        self.attach_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_client = self.create_client(DetachLink, '/GripperMagnetOFF')
        self.remove_client = self.create_client(ServoLink, '/SERVOLINK')

        # Wait for services to be available
        for client in [self.attach_client, self.detach_client, self.remove_client]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {client.srv_name} service...")

        # Service client for starting the servo
        self.servo_client = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            pass
        self.servo_request = Trigger.Request()
        
        # Initialize pose sequence
        self.pose_sequence = ['obj_1', 'obj_12', 'obj_3', 'obj_12']
        self.current_pose_index = 0

        if self.start_servo():
            time.sleep(1)
            self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
            self.timer = self.create_timer(0.008, self.publish_twist)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            
            # Record the initial pose of the first target
            self.update_target_pose()
        else:
            self.get_logger().error("Failed to start servoing")

    def start_servo(self):
        future = self.servo_client.call_async(self.servo_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().success

    def update_target_pose(self):
        """Update the target pose based on the current index in the pose sequence."""
        target_object = self.pose_sequence[self.current_pose_index]
        #try for 1 sec for the transform to be available
        for _ in range(30):
            try:
                self.target_transform = self.tf_buffer.lookup_transform('base_link', target_object, rclpy.time.Time())
                print(target_object)
                break
            except Exception as e:
                time.sleep(0.1)
                print("Waiting for transform...")

    def handle_attachment(self, action):
        """Attach or detach the box based on the current target."""
        box_name = f"box{self.pose_sequence[self.current_pose_index].split('_')[-1]}"
        if action == "attach":
            attach_request = AttachLink.Request(model1_name=box_name, link1_name='link', model2_name='ur5', link2_name='wrist_3_link')
            self.attach_client.call_async(attach_request)
            self.get_logger().info(f"Attaching {box_name} to wrist_3_link")
        elif action == "detach":
            detach_request = DetachLink.Request(model1_name=box_name, link1_name='link', model2_name='ur5', link2_name='wrist_3_link')
            self.detach_client.call_async(detach_request)
            self.get_logger().info(f"Detaching {box_name} from wrist_3_link")

            # Remove the box after detaching at drop-off
            remove_request = ServoLink.Request(box_name=box_name, box_link='link')
            self.remove_client.call_async(remove_request)
            self.get_logger().info(f"Removing {box_name}")

    def publish_twist(self):
        try:
            target_transform = self.target_transform
            effector_transform = self.tf_buffer.lookup_transform('base_link', 'wrist_3_link', rclpy.time.Time())
        except Exception as e:
            return

        # Align orientation first
        target_orientation = target_transform.transform.rotation
        effector_orientation = effector_transform.transform.rotation

        target_euler = euler_from_quaternion([target_orientation.x, target_orientation.y, target_orientation.z, target_orientation.w])
        effector_euler = euler_from_quaternion([effector_orientation.x, effector_orientation.y, effector_orientation.z, effector_orientation.w])
        delta_orientation_y = target_euler[0] - effector_euler[0]

        if math.fabs(delta_orientation_y) > 0.01:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.angular.y = -0.4 if delta_orientation_y > 0 else 0.4
            self.publisher.publish(twist_msg)
            self.get_logger().info(f"Aligning orientation: {twist_msg.twist.angular.y}")
            return

        # Move to target position if orientation is aligned
        delta_x = target_transform.transform.translation.x - effector_transform.transform.translation.x
        delta_y = target_transform.transform.translation.y - effector_transform.transform.translation.y
        delta_z = target_transform.transform.translation.z - effector_transform.transform.translation.z
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
        else:
            # Move to the next target after reaching current one
            current_target = self.pose_sequence[self.current_pose_index]
            if current_target == 'obj_12':
                self.handle_attachment("detach")
            else:
                self.handle_attachment("attach")

            # Update to the next target in the sequence
            self.current_pose_index = (self.current_pose_index + 1) % len(self.pose_sequence)
            self.update_target_pose()

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
