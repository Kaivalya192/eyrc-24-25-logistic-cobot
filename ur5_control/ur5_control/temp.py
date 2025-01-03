import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import TransformListener, Buffer
import time
import math
from std_srvs.srv import Trigger
from linkattacher_msgs.srv import AttachLink, DetachLink
from servo_msgs.srv import ServoLink
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray as Mu

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.attach_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_client = self.create_client(DetachLink, '/GripperMagnetOFF')
        self.remove_client = self.create_client(ServoLink, '/SERVOLINK')
        self.servo_client = self.create_client(Trigger, '/servo_node/start_servo')

        self.initial_rotation = None
        self.wrist_initial_position = None
        self.intermediate_poses = {}
        self.current_box = None
        self.first_sequence = True


        self.wait_for_services([self.attach_client, self.detach_client, self.remove_client, self.servo_client])

        self.pose_sequence = []
        self.sequence_captured = False
        self.sequence_completed = False
        
        self.target_transforms = {}
        self.current_pose_index = 0
        self.transforms_recorded = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        if self.start_servo():
            self.subscription = self.create_subscription(Mu, '/aruco_ids', self.aruco_ids_callback, 10)

    def aruco_ids_callback(self, msg):
        if not self.sequence_captured:
            self.pose_sequence = []
            for i in range(len(msg.data)):
                id = msg.data[i]
                if id != 12:
                    if f"obj_{id}" not in self.pose_sequence:
                        self.pose_sequence.append(f"obj_{id}")
                        self.pose_sequence.append("int")
                        self.pose_sequence.append("obj_12")
                        self.pose_sequence.append("int")
                        
            self.sequence_captured = True
            self.get_logger().info("Pose Sequence"+str(self.pose_sequence))
            
        if len(self.pose_sequence) >= 3:
            self.timer = self.create_timer(0.08, self.publish_twist)
        else:
            self.get_logger().info("No Aruco Detected")
            
            
    def wait_for_services(self, clients):
        for client in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                pass

    def start_servo(self):
        future = self.servo_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Servo started")
        return future.result() and future.result().success

    def publish_twist(self):
        if not self.sequence_captured:
            return
        if not self.transforms_recorded:
            self.record_initial_transforms()
            self.current_pose_index = 0
            return
        
        current_target = self.pose_sequence[self.current_pose_index]

        if current_target == 'int':
            prev_object = self.pose_sequence[self.current_pose_index - 1]
            next_object = self.pose_sequence[(self.current_pose_index + 1) % len(self.pose_sequence)]
            transition_key = f"{prev_object}_to_{next_object}"
            target_transform = self.intermediate_poses.get(transition_key)
        else:
            target_transform = self.target_transforms.get(current_target)
        
        if target_transform is None:
            return

        try:
            effector_transform = self.tf_buffer.lookup_transform('base_link', 'wrist_3_link', rclpy.time.Time())
        except:
            return

        # Align orientation first, then move to the target position
        if not self.align_orientation(target_transform, effector_transform):
            if not self.move_to_target(target_transform, effector_transform):
                if self.current_pose_index == len(self.pose_sequence) - 1:
                    self.get_logger().info("Pose sequence finished")
                    self.sequence_captured = False
                    self.transforms_recorded = False
                    self.first_sequence = False
                    return
                if current_target != 'int':
                    self.handle_attachment("detach" if current_target == 'obj_12' else "attach")

                self.current_pose_index = (self.current_pose_index + 1) % len(self.pose_sequence)
                self.get_logger().info(self.pose_sequence[self.current_pose_index])

    def record_initial_transforms(self):
        objects_to_record = list(set(self.pose_sequence) - {'int'}) + ['wrist_3_link']
        all_transforms_recorded = True
        for obj in objects_to_record:
            if obj not in self.target_transforms:
                if self.tf_buffer.can_transform('base_link', obj, rclpy.time.Time()):
                    try:
                        transform = self.tf_buffer.lookup_transform('base_link', obj, rclpy.time.Time())
                    except:
                        all_transforms_recorded = False
                        continue
                    self.target_transforms[obj] = transform
                    
                    # Record initial orientation for obj_5 and wrist initial position
                    if obj == self.pose_sequence[0]:
                        self.initial_rotation = transform.transform.rotation
                    elif obj == 'wrist_3_link':
                        self.wrist_initial_position = transform.transform.translation
                else:
                    print(f"Transform not available for {obj}")
                    all_transforms_recorded = False

        self.transforms_recorded = all_transforms_recorded

        # Calculate intermediate 'int' poses
        if self.transforms_recorded:
            for i in range(0, len(self.pose_sequence) - 1, 2):
                target_object = self.pose_sequence[i]
                next_object = self.pose_sequence[i + 2] if i + 2 < len(self.pose_sequence) else None
                if next_object:
                    intermediate_transform = self.tf_buffer.lookup_transform('base_link', 'wrist_3_link', rclpy.time.Time())
                    intermediate_transform.transform.rotation = self.initial_rotation
                    intermediate_transform.transform.translation = self.wrist_initial_position
                    # Move the intermediate pose 10 cm ahead on x
                    if self.first_sequence:
                        intermediate_transform.transform.translation.x += 0.1
                    
                    self.intermediate_poses[f"{target_object}_to_{next_object}"] = intermediate_transform
            if self.pose_sequence[-1] == "int":
                last_object = self.pose_sequence[-2]
                first_object = self.pose_sequence[0]
                intermediate_transform = self.tf_buffer.lookup_transform('base_link', 'wrist_3_link', rclpy.time.Time())
                intermediate_transform.transform.rotation = self.initial_rotation
                intermediate_transform.transform.translation = self.wrist_initial_position
                # Move the intermediate pose 10 cm ahead on x
                if self.first_sequence:
                    intermediate_transform.transform.translation.x += 0.1
                self.intermediate_poses[f"{last_object}_to_{first_object}"] = intermediate_transform

            self.get_logger().info("Initial transforms recorded")

            # for target_object, transform in self.target_transforms.items():
            #     print(f"{target_object}: {transform}")
            # for intermediate_key, transform in self.intermediate_poses.items():
            #     print(f"{intermediate_key}: {transform}")
            
            #edit obj_12 move 20 cm up on z
            if self.first_sequence:
                self.target_transforms['obj_12'].transform.translation.z += 0.20

    def handle_attachment(self, action):
        box_name = f"box{self.pose_sequence[self.current_pose_index].split('_')[-1]}"
        if action == "attach":
            request = AttachLink.Request(model1_name=box_name, link1_name='link', model2_name='ur5', link2_name='wrist_3_link')
            self.attach_client.call_async(request)
            self.current_box = box_name
            self.get_logger().info("attached for "+box_name)
        elif action == "detach":
            box_name = self.current_box
            request = DetachLink.Request(model1_name=box_name, link1_name='link', model2_name='ur5', link2_name='wrist_3_link')
            self.detach_client.call_async(request)
            remove_request = ServoLink.Request(box_name=box_name, box_link='link')
            self.remove_client.call_async(remove_request)
            self.get_logger().info("detached for "+box_name)

    def align_orientation(self, target_transform, effector_transform):
        target_euler = euler_from_quaternion([
            target_transform.transform.rotation.x,
            target_transform.transform.rotation.y,
            target_transform.transform.rotation.z,
            target_transform.transform.rotation.w,
        ])
        
        effector_euler = euler_from_quaternion([
            effector_transform.transform.rotation.x,
            effector_transform.transform.rotation.y,
            effector_transform.transform.rotation.z,
            effector_transform.transform.rotation.w,
        ])
        
        delta_orientation_y = target_euler[0] - effector_euler[0]
        if math.fabs(delta_orientation_y) > 0.2:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.angular.y = -5.0 if delta_orientation_y > 0 else 5.0
            self.publisher.publish(twist_msg)
            return True
        return False

    def move_to_target(self, target_transform, effector_transform):
        delta_x = target_transform.transform.translation.x - effector_transform.transform.translation.x
        delta_y = target_transform.transform.translation.y - effector_transform.transform.translation.y
        delta_z = target_transform.transform.translation.z - effector_transform.transform.translation.z
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        if distance > 0.03:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            scale_factor = 10.0 / distance
            twist_msg.twist.linear.x = scale_factor * delta_x
            twist_msg.twist.linear.y = scale_factor * delta_y
            twist_msg.twist.linear.z = scale_factor * delta_z
            self.publisher.publish(twist_msg)
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
