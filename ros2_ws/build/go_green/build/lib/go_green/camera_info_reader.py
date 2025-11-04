import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import numpy as np

class CameraInfoReader(Node):
    def __init__(self) -> None:
        super().__init__('camera_info_reader')

        
