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


        self.wait_for_services([self.attach_client, self.detach_client, self.remove_client, self.servo_client])

        self.pose_sequence = ['obj_1', 'int', 'obj_12', 'int', 'obj_2', 'int', 'obj_12']
        
        self.target_transforms = {}
        self.current_pose_index = 0
        self.transforms_recorded = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.start_servo():
            time.sleep(1)
            self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
            print("first pose")
            print(self.pose_sequence[self.current_pose_index])
            self.create_timer(0.008, self.publish_twist)

    def wait_for_services(self, clients):
        for client in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                pass

    def start_servo(self):
        future = self.servo_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().success

    def publish_twist(self):
        if not self.transforms_recorded:
            self.record_initial_transforms()
            return

        current_target = self.pose_sequence[self.current_pose_index]

        # Check if current target is an intermediate pose
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
                # Attach/detach logic only applies to main poses
                if current_target != 'int':
                    self.handle_attachment("detach" if current_target == 'obj_12' else "attach")
                # Move to the next pose in the sequence
                self.current_pose_index = (self.current_pose_index + 1) % len(self.pose_sequence)
                print("next pose")
                print(self.pose_sequence[self.current_pose_index])

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
                    intermediate_transform.transform.translation.x += 0.1
                    
                    self.intermediate_poses[f"{target_object}_to_{next_object}"] = intermediate_transform

            print("Recorded Transforms for All Objects and Intermediate Poses")

            # for target_object, transform in self.target_transforms.items():
            #     print(f"{target_object}: {transform}")
            # for intermediate_key, transform in self.intermediate_poses.items():
            #     print(f"{intermediate_key}: {transform}")
            
            #edit obj_12 move 20 cm up on z
            self.target_transforms['obj_12'].transform.translation.z += 0.25

    def handle_attachment(self, action):
        box_name = f"box{self.pose_sequence[self.current_pose_index].split('_')[-1]}"
        if action == "attach":
            request = AttachLink.Request(model1_name=box_name, link1_name='link', model2_name='ur5', link2_name='wrist_3_link')
            self.attach_client.call_async(request)
            self.current_box = box_name
            print("attached")
        elif action == "detach":
            box_name = self.current_box
            request = DetachLink.Request(model1_name=box_name, link1_name='link', model2_name='ur5', link2_name='wrist_3_link')
            self.detach_client.call_async(request)
            remove_request = ServoLink.Request(box_name=box_name, box_link='link')
            self.remove_client.call_async(remove_request)
            print("detached and removed for "+box_name)

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
        if math.fabs(delta_orientation_y) > 0.01:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.angular.y = -0.4 if delta_orientation_y > 0 else 0.4
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
            scale_factor = 0.3 / distance
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
