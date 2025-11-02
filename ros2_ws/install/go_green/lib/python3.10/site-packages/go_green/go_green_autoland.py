#!/usr/bin/env python3
"""
go_green_autoland.py
Fully autonomous green object tracking and landing using PX4 + Micro XRCE-DDS.
Uses front and downward-facing cameras to locate a green object.
Phases:
  INIT → TAKEOFF → SEARCH → APPROACH → HOVER → RETURN → LAND → DONE
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import VehicleCommand, VehicleStatus, VehicleLocalPosition, TrajectorySetpoint
import math
import time

# ---------------- CONFIGURATION ---------------- #
TAKEOFF_ALT = -3.0          # meters (NED frame)
LAND_ALT = -0.1             # final altitude before LAND
XY_GAIN = 0.5               # position correction scaling
YAW_GAIN = 0.5              # yaw correction scaling
DESCENT_RATE = 0.2          # m/s descent when aligned
CENTER_THRESHOLD = 0.1      # m — considered aligned
YAW_THRESHOLD = math.radians(5.0)  # radians
HOVER_TIME = 3.0            # seconds hover
VECTOR_TIMEOUT = 1.0        # seconds before vector considered lost
ALT_TOL = 0.2               # takeoff altitude tolerance
OFFBOARD_MODE = 6
# ------------------------------------------------ #

class GoGreenAuto(Node):
    def __init__(self):
        super().__init__("go_green_autoland")

        self.status = None
        self.pos = None
        self.current_yaw = 0.0
        self.setpoint_stream_counter = 0

        # Vector info
        self.front_vector = None
        self.down_vector = None
        self.last_front_time = 0.0
        self.last_down_time = 0.0

        # Mission state
        self.phase = "init"
        self.alt_target = TAKEOFF_ALT
        self.hover_start = None

        # ROS 2 subscriptions
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.status_cb, 10)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.pos_cb, 10)
        self.create_subscription(Vector3Stamped, "/landing_vector_front", self.front_vector_cb, 10)
        self.create_subscription(Vector3Stamped, "/landing_vector_down", self.down_vector_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)

        # Main loop timer
        self.create_timer(0.1, self.loop)
        self.get_logger().info("GoGreen AutoLand node initialized.")

    # ---------------- Callbacks ---------------- #
    def status_cb(self, msg):
        self.status = msg

    def pos_cb(self, msg):
        self.pos = msg
        self.current_yaw = msg.heading

    def front_vector_cb(self, msg):
        self.front_vector = (msg.vector.x, msg.vector.y, msg.vector.z)
        self.last_front_time = time.time()

    def down_vector_cb(self, msg):
        self.down_vector = (msg.vector.x, msg.vector.y, msg.vector.z)
        self.last_down_time = time.time()

    # ---------------- PX4 Command Helpers ---------------- #
    def arm(self, arm=True):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0 if arm else 0.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"Arm command ({'ON' if arm else 'OFF'})")

    def set_mode(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = OFFBOARD_MODE
        cmd.param2 = 0.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.cmd_pub.publish(cmd)
        self.get_logger().info("OFFBOARD mode requested")

    def land_cmd(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.cmd_pub.publish(cmd)
        self.get_logger().info("LAND command sent")

    def send_setpoint(self, x, y, z, yaw):
        sp = TrajectorySetpoint()
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = float(yaw)
        self.sp_pub.publish(sp)

    # ---------------- Mission Logic ---------------- #
    def loop(self):
        if not self.status or not self.pos:
            return

        now = time.time()

        # ---------------- INIT ---------------- #
        if self.phase == "init":
            if self.setpoint_stream_counter < 20:
                self.send_setpoint(0.0, 0.0, 0.0, self.current_yaw)
                self.setpoint_stream_counter += 1
                return

            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.set_mode()
                self.send_setpoint(0.0, 0.0, 0.0, self.current_yaw)
                return

            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm(True)
                self.send_setpoint(0.0, 0.0, 0.0, self.current_yaw)
                return

            self.get_logger().info("Armed and in Offboard mode. Phase: TAKEOFF")
            self.phase = "takeoff"
            return

        # ---------------- TAKEOFF ---------------- #
        if self.phase == "takeoff":
            if abs(self.pos.z - TAKEOFF_ALT) > ALT_TOL:
                self.send_setpoint(0.0, 0.0, TAKEOFF_ALT, self.current_yaw)
            else:
                self.phase = "search"
                self.get_logger().info("Reached takeoff altitude — starting SEARCH phase.")
            return

        # ---------------- SEARCH (Front Camera with Spiral Search) ---------------- #
        if self.phase == "search":
            if self.front_vector and now - self.last_front_time < VECTOR_TIMEOUT:
                dx, dy, yaw_err = self.front_vector
                target_x = self.pos.x + XY_GAIN * dx
                target_y = self.pos.y + XY_GAIN * dy
                target_yaw = self.current_yaw - YAW_GAIN * yaw_err

                self.send_setpoint(target_x, target_y, self.pos.z, target_yaw)

                if math.hypot(dx, dy) < CENTER_THRESHOLD:
                    self.get_logger().info("Front camera aligned — switching to APPROACH phase.")
                    self.phase = "approach"
            else:
                # Spiral search pattern
                if not hasattr(self, "search_angle"):
                    self.search_angle = 0.0
                    self.search_radius = 0.5  # start radius in meters
                    self.search_increment = 0.1  # radius increment per loop
                    self.search_speed = 0.2  # m per setpoint step

                self.search_angle += 0.1  # radians per loop
                self.search_radius += self.search_increment * 0.1  # slowly expand radius

                target_x = self.search_radius * math.cos(self.search_angle)
                target_y = self.search_radius * math.sin(self.search_angle)

                self.send_setpoint(target_x, target_y, self.pos.z, self.current_yaw)
                self.get_logger().warn_throttle(5.0, f"Searching (spiral) at x={target_x:.2f}, y={target_y:.2f}")

        # ---------------- APPROACH (Downward Camera) ---------------- #
        if self.phase == "approach":
            if self.down_vector and now - self.last_down_time < VECTOR_TIMEOUT:
                dx, dy, yaw_err = self.down_vector
                target_x = self.pos.x + XY_GAIN * dx
                target_y = self.pos.y + XY_GAIN * dy
                target_yaw = self.current_yaw - YAW_GAIN * yaw_err

                aligned_xy = math.hypot(dx, dy) < CENTER_THRESHOLD
                aligned_yaw = abs(yaw_err) < YAW_THRESHOLD

                # Descend when aligned
                if aligned_xy and aligned_yaw:
                    increment = DESCENT_RATE * 0.1  # This is a positive value (e.g., 0.02)
                    self.alt_target = self.alt_target + increment
                    # Clamp the value so it doesn't go *past* LAND_ALT
                    if self.alt_target > LAND_ALT:
                        self.alt_target = LAND_ALT
                    self.send_setpoint(self.pos.x, self.pos.y, self.alt_target, self.current_yaw)
                    if abs(self.alt_target - LAND_ALT) < ALT_TOL:
                        self.hover_start = now
                        self.phase = "hover"
                        self.get_logger().info("Reached hover altitude — starting HOVER phase.")
                else:
                    self.send_setpoint(target_x, target_y, self.alt_target, target_yaw)
            else:
                self.get_logger().warn("Downward camera lost — hovering.")
                self.send_setpoint(self.pos.x, self.pos.y, self.alt_target, self.current_yaw)
            return

        # ---------------- HOVER ---------------- #
        if self.phase == "hover":
            self.send_setpoint(self.pos.x, self.pos.y, self.alt_target, self.current_yaw)
            if now - self.hover_start > HOVER_TIME:
                self.get_logger().info("Hover complete — returning to takeoff point.")
                self.phase = "return"
            return

        # ---------------- RETURN ---------------- #
        if self.phase == "return":
            dx = -self.pos.x
            dy = -self.pos.y
            target_x = self.pos.x + XY_GAIN * dx
            target_y = self.pos.y + XY_GAIN * dy
            self.send_setpoint(target_x, target_y, TAKEOFF_ALT, self.current_yaw)
            if math.hypot(self.pos.x, self.pos.y) < CENTER_THRESHOLD:
                self.phase = "land"
                self.get_logger().info("Return complete — initiating LAND command.")
            return

        # ---------------- LAND ---------------- #
        if self.phase == "land":
            self.land_cmd()
            self.phase = "done"
            return

        # ---------------- DONE ---------------- #
        if self.phase == "done":
            self.get_logger().info_throttle(5.0, "Mission complete.")
            return


# ---------------- MAIN ---------------- #
def main():
    rclpy.init()
    node = GoGreenAuto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
