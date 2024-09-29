import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from tello_msgs.srv import TelloAction

import csv
import os
from datetime import datetime

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_sim_node')
        
        # Initialize PID variables
        self.kp = [0.003, 0.0015, 0.0005]
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
            100
        )

        # Create a publisher for drone velocity commands
        self.vel_publisher = self.create_publisher(
            Twist,
            '/drone1/cmd_vel',
            10
        )
        
        # Create a client for the Tello action service
        self.tello_action_client = self.create_client(
            TelloAction,
            '/drone1/tello_action'
        )

        # trajectory data for graphs
        self.trajectory_data = []
        self.start_time = None
        self.run_count = 0

    def tag_callback(self, msg):

        if msg.z < 10000:
            self.get_logger().info(f'Tag detected at distance: {msg.z}')
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
            Y = -float(self.kp[2] * self.y_error + self.kd[2] * (self.y_error - old_y))

            self.get_logger().info(f'Control values: FB={FB}, UD={UD}, Y={Y}')

            # Publish velocity command
            self.publish_drone_velocity(FB, 0.0, UD, Y)

            # Check if we should land
            if msg.z < 20:
                self.save_trajectory_data('square_pattern')
                self.land()
        else:
            # Publish velocity command
            self.publish_drone_velocity(0.0, 0.0, 0.0, 0.0)

    def publish_drone_velocity(self, linear_x, linear_y, linear_z, angular_z):
        vel_msg = Twist()
        vel_msg.linear.x = float(linear_x)
        vel_msg.linear.y = float(linear_y)
        vel_msg.linear.z = float(linear_z)
        vel_msg.angular.z = float(angular_z)
        self.vel_publisher.publish(vel_msg)

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.trajectory_data.append((current_time, linear_x, linear_z, angular_z))  # FB, UD, Y

    def save_trajectory_data(self, pattern_name):
        if not self.trajectory_data:
            self.get_logger().warn('No trajectory data to save')
            return

        self.run_count += 1
        filename = f"{pattern_name}_run_{self.run_count}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        directory = 'trajectory_data'
        if not os.path.exists(directory):
            os.makedirs(directory)
        
        filepath = os.path.join(directory, filename)
        
        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'FB', 'UD', 'Y'])  # Header
            writer.writerows(self.trajectory_data)
        
        self.get_logger().info(f'Trajectory data saved to {filepath}')
        self.trajectory_data = []
        self.start_time = None

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
