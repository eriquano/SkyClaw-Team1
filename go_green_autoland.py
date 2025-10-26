#!/usr/bin/env python3
"""
go_green_autoland.py
Fully autonomous precision landing using PX4 + Micro XRCE-DDS.

Phases:
  INIT → TAKEOFF → TRACK_TARGET → DESCENT → LAND → DONE
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
DESCENT_RATE = 0.2          # m/s when well-aligned
DESCENT_THRESHOLD = 0.2     # m of alignment to start descent
CENTER_THRESHOLD = 0.1      # m — stop moving if within this offset
YAW_THRESHOLD = math.radians(5.0)  # 5° yaw alignment
HOVER_TIME = 3.0            # seconds hover before LAND
VECTOR_TIMEOUT = 1.0        # seconds before vector considered lost
ALT_TOL = 0.2               # takeoff altitude tolerance
OFFBOARD_MODE = 6
# ------------------------------------------------ #


class GoGreenAuto(Node):
    def __init__(self):
        super().__init__("go_green_autoland")

        self.status = None
        self.pos = None
        self.last_vector = None
        self.last_vector_time = 0.0
        self.phase = "init"
        self.hover_start = None
        self.current_yaw = 0.0
        self.alt_target = TAKEOFF_ALT
        self.setpoint_stream_counter = 0

        # ROS 2 subscriptions (PX4 DDS topics)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.status_cb, 10)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.pos_cb, 10)
        self.create_subscription(Vector3Stamped, "/landing_vector", self.vector_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)

        self.create_timer(0.1, self.loop)
        self.get_logger().info("GoGreen AutoLand node initialized.")

    # ---------------- Callbacks ---------------- #
    def status_cb(self, msg):
        self.status = msg

    def pos_cb(self, msg):
        self.pos = msg
        self.current_yaw = msg.heading  # radians

    def vector_cb(self, msg):
        self.last_vector = (msg.vector.x, msg.vector.y, msg.vector.z)  # z = yaw error
        self.last_vector_time = time.time()

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
        cmd.param1 = 6.0  # <--- FIX: Set main mode to 6 (Offboard)
        cmd.param2 = 0.0  # Sub-mode (0 for Offboard)
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

        if self.phase == "init":
            # We must send setpoints *before* we can switch to Offboard
            # Send 20 setpoints (2 seconds) to establish the stream
            if self.setpoint_stream_counter < 20:
                self.send_setpoint(0.0, 0.0, 0.0, self.current_yaw) # Send a "hold" setpoint
                self.setpoint_stream_counter += 1
                return

            # 1. Stream established, request Offboard mode
            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.set_mode()
                self.send_setpoint(0.0, 0.0, 0.0, self.current_yaw) # Keep sending setpoints
                return

            # 2. Mode is Offboard, request Arm
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm(True)
                self.send_setpoint(0.0, 0.0, 0.0, self.current_yaw) # Keep sending setpoints
                return

            self.get_logger().info("Armed and in Offboard mode. Phase: TAKEOFF")
            self.phase = "takeoff"
            return

        # TAKEOFF phase
        if self.phase == "takeoff":
            if abs(self.pos.z - TAKEOFF_ALT) > ALT_TOL:
                self.send_setpoint(0.0, 0.0, TAKEOFF_ALT, self.current_yaw)
            else:
                self.phase = "track_target"
                self.get_logger().info("Reached takeoff altitude — starting target tracking.")
            return

        # TRACK_TARGET phase
        if self.phase == "track_target":
            if self.last_vector and now - self.last_vector_time < VECTOR_TIMEOUT:
                dx, dy, yaw_err = self.last_vector

                # Compute alignment metrics
                aligned_xy = math.hypot(dx, dy) < DESCENT_THRESHOLD
                aligned_yaw = abs(yaw_err) < YAW_THRESHOLD

                # Compute target motion
                target_x = self.pos.x + XY_GAIN * dx
                target_y = self.pos.y + XY_GAIN * dy
                target_yaw = self.current_yaw - YAW_GAIN * yaw_err

                # Begin gentle descent if well aligned
                if aligned_xy and aligned_yaw:
                    # Begin gentle descent if well aligned
                    # FIX: Add a positive increment. (DESCENT_RATE * 0.1) = 0.02
                    # alt_target goes from -3.0 -> -2.98 -> -2.96... up to LAND_ALT
                    self.alt_target = max(self.alt_target + (DESCENT_RATE * 0.1), TAKEOFF_ALT) # Failsafe
                    self.alt_target = min(self.alt_target, LAND_ALT) # <-- This is wrong
                    
                    # --- Let's use simpler logic ---
                    # FIX: Add a positive increment to move from -3.0 towards -0.1
                    increment = DESCENT_RATE * 0.1  # This is 0.02
                    self.alt_target = self.alt_target + increment
                    
                    # Clamp the value so it doesn't go past LAND_ALT
                    if self.alt_target > LAND_ALT:
                        self.alt_target = LAND_ALT

                    self.get_logger().info_throttle(1.0, f"Aligned — descending to {self.alt_target:.2f} m")
                else:
                    self.get_logger().info_throttle(
                        2.0, f"dx={dx:.2f} dy={dy:.2f} yaw_err={math.degrees(yaw_err):.1f}°")

                self.send_setpoint(target_x, target_y, self.alt_target, target_yaw)

                # Check landing condition
                if self.alt_target >= LAND_ALT:
                    self.phase = "land"
                    self.get_logger().info("At landing height — initiating LAND command.")
            else:
                # No vector input: hover
                self.get_logger().warn_throttle(5.0, "Lost target — hovering.")
                self.send_setpoint(self.pos.x, self.pos.y, self.alt_target, self.current_yaw)
                if not self.hover_start:
                    self.hover_start = now
                elif now - self.hover_start > HOVER_TIME:
                    self.phase = "land"
            return

        # LAND phase
        if self.phase == "land":
            self.land_cmd()
            self.phase = "done"
            return

        # DONE
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
