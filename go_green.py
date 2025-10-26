#!/usr/bin/env python3
"""
go_green.py — Fully autonomous PX4 ROS2 mission
Take off → find green box → hover above → return home → land
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math

# --- HSV thresholds for green ---
LOWER_GREEN = np.array([40, 40, 40])
UPPER_GREEN = np.array([80, 255, 255])

# --- Tunable parameters ---
PIXEL_TO_METER = 0.005
CENTER_TOL = 40
HOVER_TIME = 5.0
APPROACH_SPEED = 0.3
SEARCH_YAW_RATE = 0.25
TAKEOFF_ALT = 3.0
DOWNWARD_TRIGGER_ALT = 1.5
# --------------------------------


class GoGreen(Node):
    def __init__(self):
        super().__init__('go_green')
        self.bridge = CvBridge()

        # State variables
        self.state = None
        self.pose = None
        self.start_pose = None
        self.front_target = None
        self.down_target = None
        self.phase = "init"
        self.hover_start = None
        self.last_sent = 0.0

        # --- Subscribers ---
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)
        self.create_subscription(Image, '/camera_front/image_raw', self.front_cam_cb, 10)
        self.create_subscription(Image, '/camera_down/image_raw', self.down_cam_cb, 10)

        # --- Publishers ---
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # --- Service Clients ---
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("go_green node started")

    # -------------------- Callbacks -------------------- #
    def state_cb(self, msg): self.state = msg

    def pose_cb(self, msg):
        self.pose = msg
        if self.start_pose is None:
            self.start_pose = msg.pose
            self.get_logger().info("Start pose recorded.")

    def front_cam_cb(self, msg): self.front_target = self.process_image(msg, "Front Camera")
    def down_cam_cb(self, msg): self.down_target = self.process_image(msg, "Downward Camera")

    # -------------------- Image Processing -------------------- #
    def process_image(self, msg, win_name):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            cv2.imshow(win_name, cv_image); cv2.waitKey(1)
            return None

        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < 200:
            cv2.imshow(win_name, cv_image); cv2.waitKey(1)
            return None

        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w // 2, y + h // 2
        h_img, w_img = cv_image.shape[:2]
        dx, dy = cx - w_img // 2, cy - h_img // 2

        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,255,0), 2)
        cv2.circle(cv_image, (cx, cy), 5, (0,0,255), -1)
        cv2.putText(cv_image, f"dX:{dx} dY:{dy}", (cx+10, cy-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
        cv2.imshow(win_name, cv_image); cv2.waitKey(1)
        return (dx, dy)

    # -------------------- Mission Logic -------------------- #
    def loop(self):
        if not self.pose or not self.state:
            return

        # ---- INIT ----
        if self.phase == "init":
            if self.state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")
            elif not self.state.armed:
                self.arm(True)
            else:
                self.get_logger().info("Taking off...")
                self.fly_to(self.pose.pose.position.x,
                            self.pose.pose.position.y,
                            TAKEOFF_ALT)
                self.phase = "search_front"
            return

        # ---- SEARCH FRONT ----
        if self.phase == "search_front":
            if self.front_target is not None:
                self.phase = "approach_front"
                self.get_logger().info("Target detected — approaching...")
            else:
                self.rotate_in_place(SEARCH_YAW_RATE)
                self.get_logger().info_throttle(2.0, "Searching for green box...")
            return

        # ---- APPROACH FRONT ----
        if self.phase == "approach_front":
            if self.front_target is None:
                self.get_logger().warn("Lost front target, holding position.")
                self.phase = "search_front" # <-- Optionally, go back to searching
                return # Do nothing, which holds position

            dx, dy = self.front_target if self.front_target else (0, 0)
            if abs(dx) < CENTER_TOL:
                self.fly_relative(0.5 * APPROACH_SPEED, 0.0, 0.0)
                if self.pose.pose.position.z < DOWNWARD_TRIGGER_ALT and self.down_target:
                    self.phase = "refine_down"
                    self.get_logger().info("Switching to downward camera refinement.")
            else:
                # Map camera's vertical error (dy) to drone's forward/backward (x)
                # Map camera's horizontal error (dx) to drone's left/right (y)
                # The signs depend on camera orientation, but this is a common mapping:
                fwd_speed =  dy * PIXEL_TO_METER
                side_speed = -dx * PIXEL_TO_METER 
                self.fly_relative(fwd_speed, side_speed, 0.0)
            return

        # ---- REFINE DOWN ----
        if self.phase == "refine_down":
            if self.down_target is None:
                self.get_logger().info_throttle(1.0, "Lost target in downward cam.")
                return
            dx, dy = self.down_target
            if abs(dx) < CENTER_TOL and abs(dy) < CENTER_TOL:
                if not self.hover_start:
                    self.hover_start = time.time()
                    self.get_logger().info("Centered above target — hovering...")
                elif time.time() - self.hover_start > HOVER_TIME:
                    self.phase = "return_home"
                    self.get_logger().info("Hover complete. Returning home.")
            else:
                self.fly_relative(-dx * PIXEL_TO_METER, -dy * PIXEL_TO_METER, 0.0)
            return

        # ---- RETURN HOME ----
        if self.phase == "return_home":
            self.fly_to(self.start_pose.position.x,
                        self.start_pose.position.y,
                        TAKEOFF_ALT)
            dist = self.distance_to(self.start_pose.position.x,
                                    self.start_pose.position.y,
                                    TAKEOFF_ALT)
            if dist < 0.3:
                self.phase = "land"
                self.get_logger().info("Arrived home. Landing...")
            return

        # ---- LAND ----
        if self.phase == "land":
            self.set_mode("AUTO.LAND")
            self.get_logger().info_throttle(2.0, "Landing...")
            return

    # -------------------- Motion Commands -------------------- #
    def fly_relative(self, dx, dy, dz):
        """Send position offset relative to current pose."""
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = self.pose.pose.position.x + dx
        target.pose.position.y = self.pose.pose.position.y + dy
        target.pose.position.z = self.pose.pose.position.z + dz
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

    def rotate_in_place(self, yaw_rate):
        yaw = self.get_yaw(self.pose.pose.orientation)
        new_yaw = yaw + yaw_rate * 0.1
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose = self.pose.pose
        target.pose.orientation = self.yaw_to_quat(new_yaw)
        self.setpoint_pub.publish(target)

    # -------------------- Utilities -------------------- #
    def arm(self, val=True):
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Arming service not available.")
            return
        req = CommandBool.Request()
        req.value = val
        self.arming_client.call_async(req)
        self.get_logger().info("Arming command sent.")

    def set_mode(self, mode_name):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Set mode service not available.")
            return
        req = SetMode.Request()
        req.custom_mode = mode_name
        self.mode_client.call_async(req)
        self.get_logger().info(f"Mode change requested: {mode_name}")

    def get_yaw(self, q: Quaternion):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quat(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.z = math.sin(yaw / 2)
        q.x = 0.0; q.y = 0.0
        return q

    def distance_to(self, x, y, z):
        dx = self.pose.pose.position.x - x
        dy = self.pose.pose.position.y - y
        dz = self.pose.pose.position.z - z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

# -------------------- MAIN -------------------- #
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
