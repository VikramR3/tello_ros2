import cv2
from pupil_apriltags import Detector
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
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

        # Create a subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Create a publisher for AprilTag detections
        self.tag_publisher = self.create_publisher(
            Point,
            '/apriltag_detections',
            20
        )

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            tags = self.ar_detector.detect(gray_image, estimate_tag_pose=False, camera_params=None, tag_size=None)

            cv2.imshow('AprilTag Detection', cv_image)
            cv2.waitKey(1)

            if tags:
                for tag in tags:
                    dist = self.distance_calculator(tag)
                    self.get_logger().info(f'Tag {tag.tag_id} detected at distance: {dist}')

                    # Publish tag information
                    tag_msg = Point()
                    tag_msg.x = float(tag.center[0])  # x coordinate
                    tag_msg.y = float(tag.center[1])  # y coordinate
                    tag_msg.z = float(dist)  # distance
                    self.tag_publisher.publish(tag_msg)
            else:
                # Publish tag information
                tag_msg = Point()
                tag_msg.x = 0.0 # x coordinate
                tag_msg.y = 0.0  # y coordinate
                tag_msg.z = 99999.9  # distance
                self.tag_publisher.publish(tag_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

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

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
