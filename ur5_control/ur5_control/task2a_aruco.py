import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from rclpy.time import Time

def calculate_rectangle_area(coordinates):
    x = [coord[0] for coord in coordinates]
    y = [coord[1] for coord in coordinates]
    
    width = np.sqrt((x[1] - x[0])**2 + (y[1] - y[0])**2)
    height = np.sqrt((x[2] - x[1])**2 + (y[2] - y[1])**2)
    area = width * height

    return area, width

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def detect_aruco(image):
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    size_of_aruco_m = 0.15  
    aruco_area_threshold = 1500
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    corners, ids, _ = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)
    
    if ids is None:
        return [], [], [], [], []

    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []

    for i in range(len(ids)):
        coordinates = corners[i][0]
        area, width = calculate_rectangle_area(coordinates)
        if area < aruco_area_threshold:
            continue
        width_aruco_list.append(width)

        center_x = int(np.mean(coordinates[:, 0]))
        center_y = int(np.mean(coordinates[:, 1]))
        center_aruco_list.append((center_x, center_y))

        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers([coordinates], size_of_aruco_m, cam_mat, dist_mat)

        distance = np.linalg.norm(tvec[0][0])
        distance_from_rgb_list.append(distance)

        cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.1)

        rmat, _ = cv2.Rodrigues(rvec[0][0])
        angle = rotationMatrixToEulerAngles(rmat)[2]
        angle_aruco_list.append(angle)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids.flatten().tolist()

class aruco_tf(Node):
    def __init__(self):
        super().__init__('aruco_tf_publisher')                                          
        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        image_processing_rate = 0.5                                                     
        self.bridge = CvBridge()                                                        
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    
        self.timer = self.create_timer(image_processing_rate, self.process_image)       
        self.current_rgb_image = None
        self.cv_image = None                                                            
        self.depth_image = None                                                         


    def depthimagecb(self, data):
        if not hasattr(self, 'bridge'):
            self.bridge = CvBridge()
            
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')


    def colorimagecb(self, data):
        if not hasattr(self, 'bridge'):
            self.bridge = CvBridge()
        
        bgr_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.current_rgb_image = bgr_image.copy()
        self.cv_image = bgr_image.copy()

    def process_image(self):
        if self.cv_image is None or self.depth_image is None:
            return

        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        center_aruco_list, _, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.current_rgb_image)

        if not ids:
            return

        for idx, marker_id in enumerate(ids):
            cX, cY = center_aruco_list[idx]
            
            depth_at_center = self.depth_image[cY, cX] / 1000.0
            
            if depth_at_center <= 0:
                self.get_logger().warn(f"Invalid depth value for marker {marker_id}")
                continue

            distance_from_rgb = depth_at_center
            angle_aruco = angle_aruco_list[idx]
            
            roll = 1.57
            pitch = -angle_aruco
            yaw = 1.57
            
            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            
            quaternion = [qx, qy, qz, qw]
            
            original_rotation = R.from_quat(quaternion)
            camera_tilt_correction = R.from_euler('y', 0.75, degrees=False)
            corrected_rotation = camera_tilt_correction * original_rotation
            adjusted_quaternion = corrected_rotation.as_quat()

            
            x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
            y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
            z = distance_from_rgb

            cv2.circle(self.current_rgb_image, (int(cX), int(cY)), 5, (0, 255, 0), -1)

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'camera_link'
            transform.child_frame_id = f'cam_{marker_id}'
            transform.transform.translation.x = z
            transform.transform.translation.y = x
            transform.transform.translation.z = y
            transform.transform.rotation.x = adjusted_quaternion[0]
            transform.transform.rotation.y = adjusted_quaternion[1]
            transform.transform.rotation.z = adjusted_quaternion[2]
            transform.transform.rotation.w = adjusted_quaternion[3]

            self.br.sendTransform(transform)
            
            try:
                transform = self.tf_buffer.lookup_transform('base_link', f'cam_{marker_id}', Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            
            obj_transform = TransformStamped()
            obj_transform.header.stamp = self.get_clock().now().to_msg()
            obj_transform.header.frame_id = 'base_link'
            obj_transform.child_frame_id = f'obj_{marker_id}'
            obj_transform.transform = transform.transform

            self.br.sendTransform(obj_transform)

        cv2.imshow("Aruco Detection", self.current_rgb_image)
        cv2.waitKey(1)

def main():
    rclpy.init(args=sys.argv)                                       

    node = rclpy.create_node('aruco_tf_process')                    

    node.get_logger().info('Node created: Aruco tf process')        

    aruco_tf_class = aruco_tf()                                     

    rclpy.spin(aruco_tf_class)                                      

    aruco_tf_class.destroy_node()                                   

    rclpy.shutdown()                                                

if __name__ == '__main__':
    main()
