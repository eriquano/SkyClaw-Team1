
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np

# 1. ArUco Configuration
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) # Would it be better / worse to use different dimentions?
MARKER_LENGTH = 0.1  # in meters. TODO: REPLACE WITH ACTUAL MARKER SIZE

# 2. Calibrate Camera 
# TODO: REPLACE WITH ACTUAL CAMERA CALIBRATION CONSTANTS!!!!
# camera_matrix = np.array([[600, 0, 320],
#                           [0, 600, 240],
#                           [0,   0,   1]], dtype=np.float32)

# dist_coeffs = np.zeros((5, 1))  # Replace if using lens distortion

# 4. Position Storage
marker_positions = {}      # {marker_id: np.array([x, y, z])}
filtered_positions = {}    # smoothed output from low pass filter

# 5. Filter parameters
ALPHA = 0.2  # smoothing factor: 0 = very smooth, 1 = no smoothing

# 6. Marker IDs used for rectangle
RECT_MARKER_IDS = [1, 2, 3, 4, 5, 6, 7, 8]

class DrawFrame(Node):
    """Node for drawing the frame for the example"""

    def __init__(self) -> None:
        super().__init__('go_green_simple')

        # Create the puublisher for the processed video frames
        # self.video_publisher = self.create_publisher()

        # Create subscriber for video feed and info
        # TODO: replace with zed camera input
        self.camera_subscriber = self.create_subscription(
            Image, '/iris/bottom_camera/stereo_camera/left/image_raw', self.bottom_camera_callback, qos_profile=qos_profile_sensor_data)
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, '/iris/bottom_camera/stereo_camera/left/camera_info', self.camera_info_callback, 10)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

    def bottom_camera_callback(self, msg):
        """Callback function for bottom_camera topic subscriber"""

        if (self.camera_matrix is None) or (self.dist_coeffs is None):
            self.get_logger().warn_throttle(5.0, "Waiting for camera calibration...")
            return

        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # frame = cv2.resize(frame, (1280, 720))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.adaptiveThreshold(gray, 255,
                             cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                             cv2.THRESH_BINARY, 11, 2)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT)

        for r in rejected:
            r = r.astype(int)
            cv2.polylines(frame, r, isClosed=True, color=(0, 0, 255), thickness=2)

        current_frame_ids = set()

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, self.camera_matrix, self.dist_coeffs)
            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec = rvecs[i][0], tvecs[i][0]  # (3,) each
                tvec_np = np.array(tvec)

                # Low pass filter
                if marker_id in filtered_positions:
                    filtered_positions[marker_id] = (1 - ALPHA) * filtered_positions[marker_id] + ALPHA * tvec_np
                else:
                    filtered_positions[marker_id] = tvec_np.copy()

                marker_positions[marker_id] = filtered_positions[marker_id]
                current_frame_ids.add(marker_id)

                # Draw marker and axis for visualization
                cv2.aruco.drawDetectedMarkers(frame, corners)
                # cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        
        if ids is not None and len(current_frame_ids.intersection(RECT_MARKER_IDS)) >= 2:
            # Project visible marker points to image
            projected_pts = {}

            for marker_id in RECT_MARKER_IDS:
                if marker_id in current_frame_ids and marker_id in filtered_positions:
                    pt3d = filtered_positions[marker_id].reshape(1, 3)
                    rvec_zero = np.zeros((3, 1), dtype=np.float32)
                    tvec_zero = np.zeros((3, 1), dtype=np.float32)
                    pt2d, _ = cv2.projectPoints(pt3d, rvec_zero, tvec_zero, self.camera_matrix, self.dist_coeffs)
                    projected_pts[marker_id] = tuple(pt2d[0][0].astype(int))

            # Draw lines between visible consecutive markers
            for i in range(len(RECT_MARKER_IDS)):
                id1 = RECT_MARKER_IDS[i]
                id2 = RECT_MARKER_IDS[(i + 1) % len(RECT_MARKER_IDS)]
                if id1 in projected_pts and id2 in projected_pts:
                    cv2.line(frame, projected_pts[id1], projected_pts[id2], (0, 255, 0), 2)
        
        print(f"found markers: {current_frame_ids}")

        cv2.imshow("Aruco Markers", frame)
        cv2.waitKey(1)

    def camera_info_callback(self, msg):
        """gets the camera matrix for the gazebo camera"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            self.get_logger().info(f"Received camera matrix:\n{self.camera_matrix}")

def main(args=None) -> None: 
    rclpy.init(args=args)
    print('Starting go green simple node...')
    node = DrawFrame()
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

