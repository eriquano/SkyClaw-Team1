
"""idrk what its supposed to do but rn it just takes off and finds the green cube, prbly will be used for the demo"""
"""I think its supposed to take in a vector and move to the box"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from geometry_msgs.msg import Vector3Stamped
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np

#TODO: fix the movement over the box and make it go down

class GoGreen(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('go_green_script')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.camera_subscriber = self.create_subscription(
            Image, '/iris/bottom_camera/stereo_camera/left/image_raw', self.bottom_camera_callback, qos_profile=qos_profile_sensor_data)
        self.vector_subscriber = self.create_subscription(
            Vector3Stamped, 'landing_vector', self.green_vector_callback, 10)
        
        # Control Parameters
        self.kp_xy = 1.2
        self.kp_z = 1.8
        self.max_velocity_xy = 0.5
        self.max_velocity_z = 2.0
        self.position_threshold = 0.1
        self.hover_height = -1.5

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.bridge = CvBridge()

        self.desiredX = 0.0
        self.desiredY = 0.0
        self.desiredZ = self.hover_height

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0

        self.vector_valid = False
        self.last_vector_time = self.get_clock().now()

        # Wait a little bit to initialize nodes
        time.sleep(1.0)
        
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.05, self.timer_callback)

    def green_vector_callback(self, msg):
        """Callback function for landing_vector"""

        # don't do anything if box is too far away
        if msg.vector.z < 0.2 or msg.vector.z > 3.0:
            self.vector_valid = False
            return
        
        self.vector_valid = True
        self.last_vector_time = self.get_clock().now()

        offset_x = -msg.vector.x
        offset_y = -msg.vector.y

        horizontal_distance = np.sqrt(offset_x**2 + offset_y**2)

        if horizontal_distance < self.position_threshold:
            self.get_logger().info("HOLDING POSITION - Above target!")
            self.velocity_x = self.kp_xy * offset_x * 0.3
            self.velocity_y = self.kp_xy * offset_y * 0.3
            self.velocity_z = 0.0
            return

        self.velocity_x = self.kp_xy * offset_x
        self.velocity_y = self.kp_xy * offset_y

        velocity_magnitude = np.sqrt(self.velocity_x**2 + self.velocity_y**2)
        if velocity_magnitude > self.max_velocity_xy:
            self.velocity_x = self.velocity_x / velocity_magnitude * self.max_velocity_xy
            self.velocity_y = self.velocity_y / velocity_magnitude * self.max_velocity_xy

        # Altitude control - maintain hover height
        altitude_error = self.hover_height - self.vehicle_local_position.z
        # desired_distance_to_cube = 1.5
        # altitude_error = desired_distance_to_cube - msg.vector.z
        self.velocity_z = self.kp_z * altitude_error
        # self.velocity_z = np.clip(self.velocity_z, -self.max_velocity_z, self.max_velocity_z)

        self.get_logger().info(
            f"Offset (world): X={offset_x:.2f}m, Y={offset_y:.2f}m, Z={msg.vector.z:.2f}m | "
            f"Velocity cmd: vx={self.velocity_x:.2f}, vy={self.velocity_y:.2f}, vz={self.velocity_z:.2f}"
        )
        self.get_logger().info(
            f"Altitude: current_z={self.vehicle_local_position.z:.2f}m, "
            f"target={self.hover_height:.2f}m, error={altitude_error:.2f}m, vz={self.velocity_z:.2f}"
        )
        # self.get_logger().info(f"Desired vector: desiredX={self.velocity_x}, desiredY={self.velocity_y}, desiredZ={self.velocity_z}")
        # self.get_logger().info(f"Received vector: receivedX={msg.vector.x}, receivedY={msg.vector.y}, receivedZ={msg.vector.z}")

    def bottom_camera_callback(self, msg):
        """Callback function for bottom_camera topic subscriber"""
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (1280, 720))

        # just putting these here for now
        green_lower = np.array([36, 100, 100])
        green_upper = np.array([86, 255, 255])
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, green_lower, green_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # shows the green objects on screen
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Green Cube", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)



        # Display the image
        cv2.imshow('Bottom Camera', frame)
        cv2.waitKey(1)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
    
    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_heartbeat_signal(self, velocity=False):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        if velocity:
            msg.position = False
            msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float):
        """Publish velocity setpoint."""
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, vz]
        msg.yaw = 1.57079  # 90 degrees
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoints {[vx, vy, vz]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        # Let the drone take off and get stable first
        if self.offboard_setpoint_counter == 0:
            self.publish_offboard_control_heartbeat_signal()
            self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, self.hover_height)

            self.offboard_setpoint_counter += 1
            return
        elif self.offboard_setpoint_counter < 20:
            self.publish_offboard_control_heartbeat_signal()
            self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, self.hover_height)

            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()

            if self.offboard_setpoint_counter <= 10:
                self.offboard_setpoint_counter += 1
            elif self.vehicle_local_position.z < -1:
                self.offboard_setpoint_counter += 1
            return
        
        self.publish_offboard_control_heartbeat_signal(True)

        # Check if we've lost the vector
        time_since_vector = (self.get_clock().now() - self.last_vector_time).nanoseconds / 1e9
        if time_since_vector > 0.5:
            self.vector_valid = False
            self.get_logger().warn("LOST TRACKING - stopping horizontal movement")
            # Stop moving if we lose tracking
            self.velocity_x = 0.0
            self.velocity_y = 0.0
            # But maintain altitude -1 * (hover + z)
            altitude_error = self.hover_height - self.vehicle_local_position.z
            self.velocity_z = self.kp_z * altitude_error
            self.velocity_z = np.clip(self.velocity_z, -self.max_velocity_z, self.max_velocity_z)


        self.publish_velocity_setpoint(self.velocity_x, self.velocity_y, self.velocity_z)

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    green_node = GoGreen()
    try: 
        rclpy.spin(green_node)
    except KeyboardInterrupt: 
        pass
    finally: 
        green_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt: 
        pass
    except Exception as e:
        print(e)



