import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time
import math

class MarkerControl(Node):
    def __init__(self):
        super().__init__('marker_control_node')

        # Create a publisher for marker velocity commands
        self.marker_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create a timer to publish marker velocity commands
        self.marker_vel_timer = self.create_timer(0.2, self.publish_square_velocity)

    def publish_straight_velocity(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.1  # Constant velocity of 0.3 m/s
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        self.marker_vel_publisher.publish(vel_msg)

    def publish_constant_velocity(self, linear_x, linear_y, duration):
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.linear.y = linear_y
        vel_msg.angular.z = 0.0  # Ensure no rotation
        
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.marker_vel_publisher.publish(vel_msg)
            time.sleep(0.1)
        
        # Stop the movement
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        self.marker_vel_publisher.publish(vel_msg)

    def publish_circle_velocity(self):
        radius = 0.7  # Radius of the circle (in meters)
        linear_velocity = 0.5  # Linear velocity (in meters per second)
        
        circumference = 2 * math.pi * radius
        duration = circumference / linear_velocity
        
        steps = 36  # Number of steps to approximate the circle
        step_duration = duration / steps
        
        for i in range(steps):
            angle = 2 * math.pi * i / steps
            vel_x = -linear_velocity * math.sin(angle)
            vel_y = linear_velocity * math.cos(angle)
            self.publish_constant_velocity(vel_x, vel_y, step_duration)

    def publish_square_velocity(self):
        side_length = 2.0  # Length of the square side (in meters)
        velocity = 0.3  # Velocity (in meters per second)
        
        duration = side_length / velocity
        
        # Move right
        self.publish_constant_velocity(0.0, velocity, duration)
        
        # Move forward
        self.publish_constant_velocity(velocity, 0.0, duration)
        
        # Move left
        self.publish_constant_velocity(0.0, -velocity, duration)
        
        # Move backward
        self.publish_constant_velocity(-velocity, 0.0, duration)

    def publish_zigzag_velocity(self):
        velocity = 0.3  # Velocity (in meters per second)
        zigzag_width = 1.0  # Width of the zigzag (in meters)
        zigzag_height = 1.0  # Height of the zigzag (in meters)
        
        diagonal_length = math.sqrt(zigzag_width**2 + zigzag_height**2)
        duration = diagonal_length / velocity
        
        # Move diagonally up-right
        vel_x = velocity * zigzag_height / diagonal_length
        vel_y = velocity * zigzag_width / diagonal_length
        self.publish_constant_velocity(vel_x, vel_y, duration)
        
        # Move diagonally down-right
        self.publish_constant_velocity(vel_x, -vel_y, duration)
        
        # Move diagonally up-right
        self.publish_constant_velocity(vel_x, vel_y, duration)
        
        # Move diagonally down-right
        self.publish_constant_velocity(vel_x, -vel_y, duration)

    def publish_circle_velocity_rot(self):
        radius = 3.0  # Radius of the circle (in meters)
        angular_velocity = 0.2  # Angular velocity (in radians per second)

        for i in range(int(2 * math.pi / angular_velocity * 10)):
            vel_msg = Twist()
            vel_msg.linear.x = radius * angular_velocity * math.cos(angular_velocity * i / 10.0)
            vel_msg.linear.y = radius * angular_velocity * math.sin(angular_velocity * i / 10.0)
            vel_msg.angular.z = angular_velocity
            self.marker_vel_publisher.publish(vel_msg)
            time.sleep(0.1)

    def publish_square_velocity_rot(self):
        side_length = 3.0  # Length of the square side (in meters)
        velocity = 0.3  # Velocity (in meters per second)

        # Move forward
        vel_msg = Twist()
        vel_msg.linear.x = velocity
        self.marker_vel_publisher.publish(vel_msg)
        time.sleep(side_length / velocity)

        # Turn right
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = velocity / side_length
        self.marker_vel_publisher.publish(vel_msg)
        time.sleep(math.pi / 2 / (velocity / side_length))

        # Move forward
        vel_msg = Twist()
        vel_msg.linear.x = velocity
        self.marker_vel_publisher.publish(vel_msg)
        time.sleep(side_length / velocity)

        # Turn right
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = velocity / side_length
        self.marker_vel_publisher.publish(vel_msg)
        time.sleep(math.pi / 2 / (velocity / side_length))

        # Move forward
        vel_msg = Twist()
        vel_msg.linear.x = velocity
        self.marker_vel_publisher.publish(vel_msg)
        time.sleep(side_length / velocity)

        # Turn right
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = velocity / side_length
        self.marker_vel_publisher.publish(vel_msg)
        time.sleep(math.pi / 2 / (velocity / side_length))

        # Move forward
        vel_msg = Twist()
        vel_msg.linear.x = velocity
        self.marker_vel_publisher.publish(vel_msg)
        time.sleep(side_length / velocity)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
