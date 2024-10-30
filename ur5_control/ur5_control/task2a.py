from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
import tf2_ros

class ServoToAruco(Node):
    def __init__(self, marker_id):
        super().__init__('servo_to_aruco')
        self.marker_frame = f'obj_{marker_id}'
        self.effector_frame = 'ee_link'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.max_linear_speed = 0.05
        self.max_angular_speed = 0.1
        self.intermediate_steps = 5

        # Create a timer callback
        self.timer = self.create_timer(1.0, self.servo_to_target)

    def compute_intermediate_pose(self, start_pose, target_pose, step_fraction):
        # Calculate an intermediate position and orientation based on step_fraction (0 to 1)
        interp_pose = {'position': {}, 'orientation': {}}
        for axis in ['x', 'y', 'z']:
            interp_pose['position'][axis] = (1 - step_fraction) * start_pose['position'][axis] + step_fraction * target_pose['position'][axis]
            interp_pose['orientation'][axis] = (1 - step_fraction) * start_pose['orientation'][axis] + step_fraction * target_pose['orientation'][axis]
        return interp_pose
    
    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', self.effector_frame, rclpy.time.Time())
            return {
                'position': {'x': trans.transform.translation.x, 'y': trans.transform.translation.y, 'z': trans.transform.translation.z},
                'orientation': {'x': trans.transform.rotation.x, 'y': trans.transform.rotation.y, 'z': trans.transform.rotation.z}
            }
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Unable to get current end-effector pose: {str(e)}")
            return None

    def get_target_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', self.marker_frame, rclpy.time.Time())
            return {
                'position': {'x': trans.transform.translation.x, 'y': trans.transform.translation.y, 'z': trans.transform.translation.z},
                'orientation': {'x': trans.transform.rotation.x, 'y': trans.transform.rotation.y, 'z': trans.transform.rotation.z}
            }
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Unable to get target ArUco pose: {str(e)}")
            return None

    def servo_to_target(self):
        current_pose = self.get_current_pose()
        target_pose = self.get_target_pose()
        if current_pose is None or target_pose is None:
            self.get_logger().error("Could not get poses for servoing.")
            return
        else:
            self.get_logger().info("Servoing to target pose.")

        for step in range(1, self.intermediate_steps + 1):
            step_fraction = step / self.intermediate_steps
            intermediate_pose = self.compute_intermediate_pose(current_pose, target_pose, step_fraction)
            print(f"Intermediate pose: {intermediate_pose}")
            twist_cmd = TwistStamped()
            twist_cmd.twist.linear.x = (intermediate_pose['position']['x'] - current_pose['position']['x']) * self.max_linear_speed
            twist_cmd.twist.linear.y = (intermediate_pose['position']['y'] - current_pose['position']['y']) * self.max_linear_speed
            twist_cmd.twist.linear.z = (intermediate_pose['position']['z'] - current_pose['position']['z']) * self.max_linear_speed
            twist_cmd.twist.angular.x = (intermediate_pose['orientation']['x'] - current_pose['orientation']['x']) * self.max_angular_speed
            twist_cmd.twist.angular.y = (intermediate_pose['orientation']['y'] - current_pose['orientation']['y']) * self.max_angular_speed
            twist_cmd.twist.angular.z = (intermediate_pose['orientation']['z'] - current_pose['orientation']['z']) * self.max_angular_speed

            self.twist_pub.publish(twist_cmd)
            self.get_logger().info(f"Published twist command: {twist_cmd}")
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    marker_id = 6  # Replace with your marker ID
    servo_node = ServoToAruco(marker_id)
    rclpy.spin(servo_node)
    servo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
