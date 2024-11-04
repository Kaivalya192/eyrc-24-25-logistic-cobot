import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from builtin_interfaces.msg import Time
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            pass
        self.req = Trigger.Request()
        
        if self.start_servo():
            time.sleep(1)  
            self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
            self.timer = self.create_timer(0.008, self.publish_twist)
        else:
            self.get_logger().error("Failed to start servoing")

    def start_servo(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().success

    def publish_twist(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg() 
        twist_msg.header.frame_id = 'base_link' 
        twist_msg.twist.linear.z = 0.2
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
