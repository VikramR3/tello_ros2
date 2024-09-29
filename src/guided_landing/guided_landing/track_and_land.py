import cv2
from pupil_apriltags import Detector
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# Import the custom Tello action service
from tello_msgs.srv import TelloAction


class AprilTagDetector:
    def __init__(self, node):
        self.node = node
        self.cv_bridge = CvBridge()
        
        # Create the AprilTag detector
        self.ar_detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # Initialize PID variables
        self.kp = [0.0003, 0.00015, 0.0]  # [0.3, 0.08, 0.1] 
        self.kd = [0.0, 0.0, 0.0]  # [3, 2, 1.5]
        
        # Initialize error variables
        self.lr_error = 0
        self.ud_error = 0
        self.fb_error = 0
        self.y_error = 0
        
        self.latest_image = None
        
    def image_callback(self, msg):
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.node.get_logger().error(f'Error converting image: {str(e)}')

    def process_image(self):
        if self.latest_image is None:
            self.node.get_logger().warn('No image received yet')
            return
        
        targets = [0, 1, 2]
        control_values = [0, 0, 0, 0]
        should_land = 0

        cv_image = self.latest_image
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        tags = self.ar_detector.detect(gray_image, estimate_tag_pose=False, camera_params=None, tag_size=None)

        # remove after testing
        cv2.waitKey(1)
        cv2.imshow('Test 2', gray_image)

        if not tags:
            return control_values
        
        for tag in tags:
            if targets.__contains__(tag.tag_id):
                self.node.get_logger().warn('Found tag')

                dist = self.distance_calculator(tag)
                self.node.get_logger().info(f'Tag {tag.tag_id} detected at distance: {dist}')

                # Update error values
                old_lr = self.lr_error
                old_ud = self.ud_error
                old_fb = self.fb_error
                old_y = self.y_error
                self.lr_error = tag.center[0] - 640
                self.ud_error = (tag.center[1] - 360) * -1
                self.fb_error = dist
                self.y_error = tag.center[0] - 600

                # Calculate control values
                FB = float(self.kp[0] * self.fb_error + self.kd[0] * (self.fb_error - old_fb))
                UD = float(self.kp[1] * self.ud_error + self.kd[1] * (self.ud_error - old_ud))
                Y = float(self.kp[2] * self.y_error + self.kd[2] * (self.y_error - old_y))

                if dist < 20:
                    should_land = 1

                control_values = [FB, UD, Y, should_land]
                self.node.get_logger().info(f'Control values: FB={FB}, UD={UD}, Y={Y}')

        return control_values

    def distance_calculator(self, tag):
        one_two_length = math.sqrt(
            math.pow((tag.corners[0][0] - tag.corners[1][0]), 2) + math.pow((tag.corners[0][1] - tag.corners[1][1]), 2))
        two_three_length = math.sqrt(
            math.pow((tag.corners[1][0] - tag.corners[2][0]), 2) + math.pow((tag.corners[1][1] - tag.corners[2][1]), 2))
        three_four_length = math.sqrt(
            math.pow((tag.corners[2][0] - tag.corners[3][0]), 2) + math.pow((tag.corners[2][1] - tag.corners[3][1]), 2))
        four_one_length = math.sqrt(
            math.pow((tag.corners[3][0] - tag.corners[0][0]), 2) + math.pow((tag.corners[3][1] - tag.corners[0][1]), 2))
        avg_length = (one_two_length + two_three_length + three_four_length + four_one_length) / 4
        dist = 5703 * pow(avg_length, -1.02)
        return dist

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
        
        # Create a subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',  # Replace with your actual image topic
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a publisher for drone velocity commands
        self.vel_publisher = self.create_publisher(
            Twist,
            '/drone1/cmd_vel',
            10
        )
        
        # Create a publisher for marker velocity commands
        self.marker_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create a client for the Tello action service
        self.tello_action_client = self.create_client(
            TelloAction,
            '/drone1/tello_action'
        )

        # Create a timer to publish marker velocity commands
        self.marker_vel_timer = self.create_timer(0.1, self.publish_marker_velocity)
        
        self.detector = AprilTagDetector(self)

    def publish_marker_velocity(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.3  # Constant velocity of 0.3 m/s
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        self.marker_vel_publisher.publish(vel_msg)

    def image_callback(self, msg):
        self.detector.image_callback(msg)

    def publish_velocity(self, linear_x, linear_y, linear_z, angular_z):
        vel_msg = Twist()
        vel_msg.linear.x = float(linear_x)
        vel_msg.linear.y = float(linear_y)
        vel_msg.linear.z = float(linear_z)
        vel_msg.angular.z = float(angular_z)
        self.vel_publisher.publish(vel_msg)
    
    def takeoff(self):
        # Wait for the service to become available
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
        
        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Takeoff command sent successfully')
            return True
        else:
            self.get_logger().error('Failed to send takeoff command')
            return False
        
    def land(self):
        # Wait for the service to become available
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
        
        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Land command sent successfully')
            return True
        else:
            self.get_logger().error('Failed to send land command')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    node.takeoff()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            control_values = node.detector.process_image()
            if control_values:
                FB, UD, Y, should_land = control_values
                node.get_logger().info(f'Control values: FB={FB}, UD={UD}, Y={Y}')
                
                # Publish velocity command
                node.publish_velocity(
                    linear_x=float(FB),  # Forward/Backward
                    linear_y=0.0,        # Left/Right (not used in this example)
                    linear_z=float(UD),  # Up/Down
                    angular_z=float(Y)   # Yaw
                )

                # Do something with the control values
                node.get_logger().info(f'Using control values: {control_values}')

                if should_land:
                    if node.land():
                        break
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()