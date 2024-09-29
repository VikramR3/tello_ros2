import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from tello_msgs.srv import TelloAction

import time
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        
        # Initialize PID variables
        self.kp = [0.003, 0.0015, 0.0]
        self.kd = [0.0, 0.0, 0.0]
        
        # Initialize error variables
        self.lr_error = 0
        self.ud_error = 0
        self.fb_error = 0
        self.y_error = 0
        
        # Create a subscription to the AprilTag detections
        self.subscription = self.create_subscription(
            Point,
            '/apriltag_detections',
            self.tag_callback,
            20
        )

        # Create a publisher for drone velocity commands
        self.vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create a client for the Tello action service
        self.tello_action_client = self.create_client(
            TelloAction,
            '/tello_action'
        )

    def tag_callback(self, msg):
        # Update error values
        old_lr = self.lr_error
        old_ud = self.ud_error
        old_fb = self.fb_error
        old_y = self.y_error
        
        self.lr_error = msg.x - 640
        self.ud_error = (msg.y - 360) * -1
        self.fb_error = msg.z
        self.y_error = msg.x - 600

        # Calculate control values
        FB = float(self.kp[0] * self.fb_error + self.kd[0] * (self.fb_error - old_fb))
        UD = float(self.kp[1] * self.ud_error + self.kd[1] * (self.ud_error - old_ud))
        Y = float(self.kp[2] * self.y_error + self.kd[2] * (self.y_error - old_y))

        self.get_logger().info(f'Control values: FB={FB}, UD={UD}, Y={Y}')

        # Publish velocity command
        self.publish_velocity(FB, 0.0, UD, Y)

        # Check if we should land
        if msg.z < 20:
            self.land()

    def publish_velocity(self, linear_x, linear_y, linear_z, angular_z):
        vel_msg = Twist()
        vel_msg.linear.x = float(linear_x)
        vel_msg.linear.y = float(linear_y)
        vel_msg.linear.z = float(linear_z)
        vel_msg.angular.z = float(angular_z)
        self.vel_publisher.publish(vel_msg)

    def takeoff(self):
        self.get_logger().info('Waiting for Tello action service...')
        while not self.tello_action_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                return False
            self.get_logger().info('Tello action service not available, waiting...')

        request = TelloAction.Request()
        request.cmd = 'takeoff'
        
        self.get_logger().info('Sending takeoff command...')
        future = self.tello_action_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Takeoff command sent successfully')
            return True
        else:
            self.get_logger().error('Failed to send takeoff command')
            return False

    def land(self):
        self.get_logger().info('Waiting for Tello action service...')
        while not self.tello_action_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                return False
            self.get_logger().info('Tello action service not available, waiting...')

        request = TelloAction.Request()
        request.cmd = 'land'
        
        self.get_logger().info('Sending land command...')
        future = self.tello_action_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Land command sent successfully')
            return True
        else:
            self.get_logger().error('Failed to send land command')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    
    if node.takeoff():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
