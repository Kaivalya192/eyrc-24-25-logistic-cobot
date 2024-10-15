import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

class EndEffectorPosition(Node):
    def __init__(self):
        super().__init__('end_effector_position')
        
        # Create a TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a subscription to the joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',  # Typically the topic where joint states are published
            self.joint_state_callback,
            10
        )
        
        # Dictionary to store the latest joint states
        self.joint_states = {}
        
        # Call the function periodically every second to print the position and orientation
        self.timer = self.create_timer(1.0, self.print_position_and_orientation)
        
    def joint_state_callback(self, msg: JointState):
        # Update the joint states with the latest values
        for name, position in zip(msg.name, msg.position):
            self.joint_states[name] = position

    def print_position_and_orientation(self):
        try:
            # Lookup transform from 'base_link' to 'ee_link'
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',  # Reference frame
                'ee_link',    # Target frame
                rclpy.time.Time()  # Get the latest transform
            )
            
            # Extract position
            translation = transform.transform.translation
            # Extract orientation (as quaternion)
            rotation = transform.transform.rotation

            # Format position, orientation, and joint angles
            position = [translation.x, translation.y, translation.z]
            orientation = [rotation.x, rotation.y, rotation.z, rotation.w]
            joint_angles = [
                self.joint_states.get('shoulder_pan_joint', 'N/A'),
                self.joint_states.get('shoulder_lift_joint', 'N/A'),
                self.joint_states.get('elbow_joint', 'N/A'),
                self.joint_states.get('wrist_1_joint', 'N/A'),
                self.joint_states.get('wrist_2_joint', 'N/A'),
                self.joint_states.get('wrist_3_joint', 'N/A')
            ]

            # Print in the required format
            self.get_logger().info(
                f'([{position[0]}, {position[1]}, {position[2]}], \n'
                f'[{orientation[0]}, {orientation[1]}, {orientation[2]}, {orientation[3]}],\n'
                f'[{joint_angles[0]}, {joint_angles[1]}, {joint_angles[2]}, '
                f'{joint_angles[3]}, {joint_angles[4]}, {joint_angles[5]}])'
                f'\n'
            )
        
        except Exception as e:
            self.get_logger().error(f'Could not transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
