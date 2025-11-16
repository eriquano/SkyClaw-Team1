
"""Supposed to subscribe to a camera feed and publish both a vector and a camera stream with drawings on it"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

import numpy as np
import cv2
from collections import deque
import math

# ------------------ CONFIGURATION ------------------ #
LOWER_GREEN = np.array([40, 40, 40])
UPPER_GREEN = np.array([80, 255, 255])

MIN_AREA = 500
ASPECT_RATIO_MIN = 0.75
ASPECT_RATIO_MAX = 1.50
MATCH_DISTANCE = 50
PERSISTENCE_THRESHOLD = 5
PIXEL_TO_METER = 0.005
# --------------------------------------------------- #

class GreenTracker(Node):
    
    def __init__(self):
        super().__init__('green_tracker_publisher')


        # Create publishers
        self.green_tracker_vector_publisher = self.create_publisher(
            Vector3Stamped, 'landing_vector', 10)
        self.green_tracker_camera_publisher = self.create_publisher(
            Image, 'camera_feed', 10)
        
        # Create subscribers
        self.camera_subscriber_left = self.create_subscription(
            Image, '/iris/bottom_camera/stereo_camera/left/image_raw', self.left_callback, qos_profile=qos_profile_sensor_data)        
        self.camera_subscriber_right = self.create_subscription(
            Image, '/iris/bottom_camera/stereo_camera/right/image_raw', self.right_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/iris/bottom_camera/stereo_camera/left/camera_info', self.left_info_callback, 10)
        self.create_subscription(CameraInfo, '/iris/bottom_camera/stereo_camera/right/camera_info', self.right_info_callback, 10)

        # Initialize variables
        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.left_image = cv2.resize(self.left_image, (1280, 720))

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.right_image = cv2.resize(self.right_image, (1280, 720))

    def left_info_callback(self, msg):
        self.left_info = msg

    def right_info_callback(self, msg):
        self.right_info = msg

    def publish_green_vector(self, x: float, y: float, z: float):
        """Publish the green vector."""
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "bottom_camera"   # or whatever your frame is

        msg.vector.x = float(x)
        msg.vector.y = float(y)
        msg.vector.z = float(z)

        self.green_tracker_vector_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        if self.left_image is None or self.right_image is None:
            return

        # just putting these here for now
        green_lower = np.array([36, 100, 100])
        green_upper = np.array([86, 255, 255])
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))

        hsv_left = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2HSV)
        mask_left = cv2.inRange(hsv_left, green_lower, green_upper)
        mask_left = cv2.morphologyEx(mask_left, cv2.MORPH_OPEN, kernel)

        hsv_right = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2HSV)
        mask_right = cv2.inRange(hsv_right, green_lower, green_upper)
        mask_right = cv2.morphologyEx(mask_right, cv2.MORPH_OPEN, kernel)

        #idk what anything after this is
        M_left = cv2.moments(mask_left)
        M_right = cv2.moments(mask_right)

        if M_left["m00"] > 0 and M_right["m00"] > 0:
            cx_left = int(M_left["m10"] / M_left["m00"])
            cy_left = int(M_left["m01"] / M_left["m00"])
            cx_right = int(M_right["m10"] / M_right["m00"])

            disparity = cx_left - cx_right
            if disparity > 0.1:
                fx = self.left_info.k[0]  # focal length in pixels
                cx = self.left_info.k[2]
                cy = self.left_info.k[5]
                B = 0.12  # baseline (in meters, adjust to your SDF)

                Z = (fx * B) / disparity
                X = (cx_left - cx) * Z / fx
                Y = (cy_left - cy) * Z / fx

                self.get_logger().info(f"3D Vector to object: X={X:.2f}m, Y={Y:.2f}m, Z={Z:.2f}m")

                self.publish_green_vector(X, Y, Z)


    

def main():
    print('Starting green tracker node...')
    rclpy.init(args=None)
    pub = GreenTracker()
    try: 
        rclpy.spin(pub)
    except KeyboardInterrupt: 
        pass
    finally: 
        pub.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
