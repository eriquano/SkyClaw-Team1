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
        self.hover_height = -1.5

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.bridge = CvBridge()

        # Target tracking
        self.target_position = None
        self.target_locked = False
        self.vector_samples = []
        self.samples_needed = 500  # We can sample more if needed, samples at 100 Hz
        self.collecting_samples = False
        self.allow_sampling = False

        # Wait a little bit to initialize nodes
        time.sleep(1.0)
        
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.05, self.timer_callback)

    def green_vector_callback(self, msg):
        """Callback function for landing_vector"""

        # Don't collect if we already have a target locked
        if self.target_locked:
            return
        
        # Don't do anything until we reach hover height
        if not self.allow_sampling:
            return

        # Don't do anything if box is too far away
        if msg.vector.z < 0.2 or msg.vector.z > 3.0:
            return
        
        # Start collecting samples
        if not self.collecting_samples:
            self.collecting_samples = True
            self.vector_samples = []
            self.get_logger().info("Started collecting position samples...")

        # Add sample (offset in camera frame)
        offset_x = -msg.vector.x
        offset_y = -msg.vector.y
        self.vector_samples.append([offset_x, offset_y])

        # Check if we have enough samples
        if len(self.vector_samples) >= self.samples_needed:
            # Calculate average offset
            avg_offset = np.mean(self.vector_samples, axis=0)
            
            # Convert to world position (add to current drone position)
            target_x = self.vehicle_local_position.x + avg_offset[0]
            target_y = self.vehicle_local_position.y + avg_offset[1]
            target_z = self.hover_height
            
            self.target_position = (target_x, target_y, target_z)
            self.target_locked = True
            self.collecting_samples = False
            
            self.get_logger().info(
                f"TARGET LOCKED! Moving to: X={target_x:.2f}m, Y={target_y:.2f}m, Z={target_z:.2f}m"
            )
            self.get_logger().info(
                f"(Averaged {len(self.vector_samples)} samples, offset: X={avg_offset[0]:.2f}m, Y={avg_offset[1]:.2f}m)"
            )
        else:
            # Log progress
            if len(self.vector_samples) % 20 == 0:
                self.get_logger().info(f"Collecting samples: {len(self.vector_samples)}/{self.samples_needed}")

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
            
            # Show status
            if self.target_locked:
                cv2.putText(frame, "TARGET LOCKED", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            elif self.collecting_samples:
                cv2.putText(frame, f"Sampling {len(self.vector_samples)}/{self.samples_needed}", 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
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

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
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
        self.publish_offboard_control_heartbeat_signal()
        
        # Initial takeoff sequence
        if self.offboard_setpoint_counter < 20:
            self.publish_position_setpoint(
                self.vehicle_local_position.x, 
                self.vehicle_local_position.y, 
                self.hover_height
            )

            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()

            if self.offboard_setpoint_counter <= 10:
                self.offboard_setpoint_counter += 1
            elif self.vehicle_local_position.z < -1:
                self.offboard_setpoint_counter += 1
            return
        
        # Check if we're at hover height (within 10cm)
        at_hover_height = abs(self.vehicle_local_position.z - self.hover_height) < 0.1
        
        if not at_hover_height and not self.target_locked:
            # Still climbing to hover height - don't collect samples yet
            self.publish_position_setpoint(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.hover_height
            )
            if self.offboard_setpoint_counter % 20 == 0:
                self.get_logger().info(f"Climbing to hover height... (current: {self.vehicle_local_position.z:.2f}m, target: {self.hover_height:.2f}m)")
            self.offboard_setpoint_counter += 1
            return
        
        if not self.target_locked:
            self.allow_sampling = True
        
        # If we have a target, fly to it
        if self.target_locked and self.target_position is not None:
            self.publish_position_setpoint(
                self.target_position[0],
                self.target_position[1],
                self.target_position[2]
            )
            
            # Calculate distance to target
            dx = self.target_position[0] - self.vehicle_local_position.x
            dy = self.target_position[1] - self.vehicle_local_position.y
            dz = self.target_position[2] - self.vehicle_local_position.z
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # Log progress every 20 cycles
            if self.offboard_setpoint_counter % 20 == 0:
                self.get_logger().info(f"Distance to target: {distance:.2f}m")
        else:
            # At hover height but no target yet - just hover and wait for samples to collect
            self.publish_position_setpoint(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.hover_height
            )
        
        self.offboard_setpoint_counter += 1

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