from linkattacher_msgs.srv._attach_link import AttachLink
from linkattacher_msgs.srv._detach_link import DetachLink
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose
import time as Time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            ],
            base_link_name='base_link',
            end_effector_name='ee_link',
            group_name='ur_manipulator'
        )

        self.positions = {
            'P1': ([0.20, -0.47, 0.65], 
                    [0.7079887004926899, -0.01361514447882504, -0.015041124186426811, 0.7059322859869077],
                    [-1.3150506155519546, -1.48584249511307, 1.3600472209607872, -3.0128000376273794, -1.8670726792029069, 3.1403926005709417]),
            'P3': ([0.75,0.49,-0.05], 
                    [-0.7008651442731405, 0.7131082301805051, 0.015214678576058087, 0.005763258422517251],
                    [0.35672716303061147, -0.336091994400745, 0.8329299672928787, -2.0657499918920252, -1.5383101689905951, 0.37401339969258673]),
            'P2': ([0.75,-0.23,-0.0499], 
                    [0.6742704038407381, 0.7383053938884049, 0.0021900713551518, -0.01611742681679008],
                    [-0.4468265043229893, -0.47412148360071305, 1.1183207414070733, -2.185400172463661, -1.5572859733607027, -3.6792212168362153]),
            'D': ([-0.69, 0.10, 0.44], 
                    [-0.48604999712858793, -0.5079208857004494, 0.5204362366834104, 0.4846832962949929],
                    [2.7961322425440827, -0.9826515070982702, 1.2505999195985558, -3.4220901853590235, -1.167808479217307, 0.01847743179350303])
            }

        self.move_to_positions(['P1','D','P3','D','P2','D'])

    def move_to_positions(self, sequence):
        for position_name in sequence:
            self.get_logger().info(f'Moving to {position_name}...')
            position, orientation, *joints = self.positions[position_name]
            if joints:
                self.move_to_joint_positions(joints[0])
            else:
                target_pose = self.create_pose(position, orientation)
                self.move_to_pose(target_pose)
            self.get_logger().info(f'Moved to {position_name}.')

            
            if position_name == 'P1':
                box_name = "box1"
            elif position_name == 'P2':
                box_name = "box49"
            elif position_name == 'P3':
                box_name = "box3"
            else:
                box_name = None

            
            if box_name:
                self.attach_gripper(box_name)
                self.last_attached_box = box_name
            elif position_name == 'D' and self.last_attached_box:
                self.detach_gripper(self.last_attached_box)
                self.last_attached_box = None


    def move_to_joint_positions(self, joint_angles):
        self.moveit2.move_to_configuration(
            joint_positions=joint_angles,
            tolerance=0.05
        )
        self.get_logger().info("Waiting for execution to complete...")
        Time.sleep(4)
        self.get_logger().info("Execution completed.")

    def attach_gripper(self, box_name):
        self.get_logger().info(f"Attaching gripper to {box_name}...")
        future = self.create_client(
            AttachLink, '/GripperMagnetON'
        ).call_async(AttachLink.Request(
            model1_name=box_name,
            link1_name='link',
            model2_name='ur5',
            link2_name='wrist_3_link'
        ))
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Gripper attached to {box_name}.")
        else:
            self.get_logger().error(f"Failed to attach the gripper to {box_name}.")

    def detach_gripper(self, box_name):
        self.get_logger().info(f"Detaching gripper from {box_name}...")
        future = self.create_client(
            DetachLink, '/GripperMagnetOFF'
        ).call_async(DetachLink.Request(
            model1_name=box_name,
            link1_name='link',
            model2_name='ur5',
            link2_name='wrist_3_link'
        ))
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Gripper detached from {box_name}.")
        else:
            self.get_logger().error(f"Failed to detach the gripper from {box_name}.")

    def create_pose(self, position, orientation):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        return pose

    def move_to_pose(self, pose):
        self.moveit2.move_to_pose(
            position=[pose.position.x, pose.position.y, pose.position.z],
            quat_xyzw=[
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ],
            cartesian=False,
            tolerance_position=0.05,
            tolerance_orientation=0.05,
            weight_position=2.0,
            weight_orientation=2.0
        )
        self.get_logger().info("Waiting for execution to complete...")
        Time.sleep(8)
        self.get_logger().info("Execution completed.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()