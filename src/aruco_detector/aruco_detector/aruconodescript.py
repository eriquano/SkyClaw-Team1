# ROS2/Python Imports
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion
from aruco_interfaces.msg import MarkerArray, MarkerPose, LineData
from scipy.spatial.transform import Rotation as R
import cv2
import numpy as np
import os

class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer')
        self.get_logger().info('✅ Aruco Localizer node has been started.')

        # === SETUP VARIABLES ===
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.marker_length = 0.15  # in meters
        self.camera_matrix = np.array([[600.0, 0.0, 320.0],
                                       [0.0, 600.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros((5, 1))
        self.filtered_positions = {}
        self.alpha = 0.2
        self.rect_marker_ids = [1, 2, 3, 4, 5, 6, 7, 8]

        # === ROS2 SUBSCRIBERS & PUBLISHERS ===
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco/markers', 10)
        self.line_data_pub = self.create_publisher(LineData, '/aruco/line_data', 10)
        self.image_pub = self.create_publisher(Image, '/aruco/detection_image', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        current_frame_ids = set()
        marker_array_msg = MarkerArray()
        marker_array_msg.header = msg.header

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            
            for i, marker_id_np in enumerate(ids):
                marker_id = int(marker_id_np[0])
                rvec, tvec = rvecs[i][0], tvecs[i][0]
                
                if marker_id in self.filtered_positions:
                    self.filtered_positions[marker_id] = (1 - self.alpha) * self.filtered_positions[marker_id] + self.alpha * tvec
                else:
                    self.filtered_positions[marker_id] = tvec.copy()
                
                filtered_tvec = self.filtered_positions[marker_id]
                current_frame_ids.add(marker_id)
                
                marker_pose_msg = self.create_marker_pose_msg(marker_id, rvec, filtered_tvec)
                marker_array_msg.markers.append(marker_pose_msg)

                cv2.aruco.drawDetectedMarkers(cv_image, corners)
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

            if marker_array_msg.markers:
                self.marker_pub.publish(marker_array_msg)

        if ids is not None and len(current_frame_ids.intersection(self.rect_marker_ids)) >= 2:
            self.process_and_publish_line_data(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')

    def create_marker_pose_msg(self, marker_id, rvec, tvec):
        marker_pose = MarkerPose()
        marker_pose.marker_id = marker_id
        marker_pose.position.x, marker_pose.position.y, marker_pose.position.z = tvec
        rotation_matrix = cv2.Rodrigues(rvec)[0]
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()
        marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w = quat
        return marker_pose

    def process_and_publish_line_data(self, image_to_draw_on):
        projected_pts = {}
        visible_rect_ids = set(self.filtered_positions.keys()).intersection(self.rect_marker_ids)

        for marker_id in visible_rect_ids:
            pt3d = self.filtered_positions[marker_id].reshape(1, 3)
            pt2d, _ = cv2.projectPoints(pt3d, np.zeros((3,1)), np.zeros((3,1)), self.camera_matrix, self.dist_coeffs)
            projected_pts[marker_id] = tuple(pt2d[0][0].astype(int))

        for i in range(len(self.rect_marker_ids)):
            id1 = self.rect_marker_ids[i]
            id2 = self.rect_marker_ids[(i + 1) % len(self.rect_marker_ids)]
           
            if id1 in projected_pts and id2 in projected_pts:
                cv2.line(image_to_draw_on, projected_pts[id1], projected_pts[id2], (0, 255, 0), 2)
                
                p1_3d = self.filtered_positions[id1]
                p2_3d = self.filtered_positions[id2]
                length = np.linalg.norm(p1_3d - p2_3d)
                midpoint_3d = (p1_3d + p2_3d) / 2

                line_msg = LineData()
                line_msg.marker_ids = [id1, id2]
                line_msg.length_m = float(length)
                line_msg.midpoint.x, line_msg.midpoint.y, line_msg.midpoint.z = midpoint_3d
                self.line_data_pub.publish(line_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
