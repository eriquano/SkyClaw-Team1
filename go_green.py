#!/usr/bin/env python3
"""
go_green.py — Autonomous PX4 mission:
Locate green box → hover above → return to start
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

LOWER_GREEN = np.array([40, 40, 40])
UPPER_GREEN = np.array([80, 255, 255])
PIXEL_TO_METER = 0.005  # rough scale factor — tune for your camera setup
APPROACH_SPEED = 0.5
HOVER_TIME = 5.0
CENTER_TOL = 40  # pixels tolerance for "aligned"

class GoGreen(Node):
    def __init__(self):
        super().__init__('go_green')

        self.bridge = CvBridge()
        self.state = None
        self.pose = None
        self.start_pose = None
        self.target_found = False
        self.hovering = False
        self.hover_start = None

        # Subscribers
        self.create_subscription(Image, '/camera_down/image_raw', self.image_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)

        # Publisher for position control
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("go_green node started")

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.pose = msg
        if self.start_pose is None:
            self.start_pose = msg.pose
            self.get_logger().info("Start position recorded")

    def image_cb(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

        # Find largest contour (green box)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.target_found = False
            return

        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < 200:
            self.target_found = False
            return

        x, y, w, h = cv2.boundingRect(cnt)
        cx = x + w // 2
        cy = y + h // 2

        h_img, w_img = cv_image.shape[:2]
        dx = cx - w_img // 2
        dy = cy - h_img // 2

        self.target_found = True
        self.target_offset = (dx, dy)

        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,255,0), 2)
        cv2.circle(cv_image, (cx, cy), 5, (0,0,255), -1)
        cv2.imshow("Downward Camera", cv_image)
        cv2.waitKey(1)

    def control_loop(self):
        if self.pose is None or self.start_pose is None:
            return

        if not self.target_found:
            self.get_logger().info_throttle(2.0, "Searching for green box...")
            return

        dx_px, dy_px = self.target_offset
        if abs(dx_px) < CENTER_TOL and abs(dy_px) < CENTER_TOL:
            # Hover
            if not self.hovering:
                self.hovering = True
                self.hover_start = time.time()
                self.get_logger().info("Hovering above target...")
            elif time.time() - self.hover_start > HOVER_TIME:
                self.get_logger().info("Returning to start position...")
                self.fly_to(self.start_pose.position.x,
                            self.start_pose.position.y,
                            self.start_pose.position.z)
        else:
            # Move toward green box
            dx = -dx_px * PIXEL_TO_METER
            dy = -dy_px * PIXEL_TO_METER
            self.fly_relative(dx, dy)

    def fly_relative(self, dx, dy):
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = self.pose.pose.position.x + dx
        target.pose.position.y = self.pose.pose.position.y + dy
        target.pose.position.z = self.pose.pose.position.z
        target.pose.orientation = self.pose.pose.orientation
        self.setpoint_pub.publish(target)

    def fly_to(self, x, y, z):
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation = self.pose.pose.orientation
        self.setpoint_pub.publish(target)

def main():
    rclpy.init()
    node = GoGreen()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
