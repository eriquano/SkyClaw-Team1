import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import imutils
import pyzed.sl as sl
import time

def pose_estimation(frame, detector, matrix_coefficients, distortion_coefficients, marker_size=0.025):
    # Detect markers
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None and len(ids) > 0:
        # Draw detected markers
        cv.aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            c = corners[i][0]  # 4 corner points

            # 3D points of the marker in local coordinates
            obj_points = np.array([
                [-marker_size/2,  marker_size/2, 0],
                [ marker_size/2,  marker_size/2, 0],
                [ marker_size/2, -marker_size/2, 0],
                [-marker_size/2, -marker_size/2, 0]
            ], dtype=np.float32)

            # --- FIX IS HERE ---
            # Correctly unpack the return values: success, rvec, tvec
            success, rvec, tvec = cv.solvePnP(obj_points, c, matrix_coefficients, distortion_coefficients)

            # Check if the pose was successfully estimated
            if success:
                # Draw axes
                cv.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.02)

                # Draw marker ID
                center = np.mean(c, axis=0).astype(int)
                cv.putText(frame, f"ID: {ids[i][0]}", tuple(center),
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame

# --- Initialize ZED Camera ---
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.depth_mode = sl.DEPTH_MODE.NONE  # RGB only
init_params.coordinate_units = sl.UNIT.METER

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("❌ Failed to open ZED camera")
    exit(1)

# --- Get Calibration (intrinsics) ---
cam_info = zed.get_camera_information()
calib = cam_info.camera_configuration.calibration_parameters.left_cam

fx, fy, cx, cy = calib.fx, calib.fy, calib.cx, calib.cy
cameraMatrix = np.array([[fx, 0, cx],
                         [0, fy, cy],
                         [0, 0, 1]], dtype=np.float32)
distortion = np.array(calib.disto, dtype=np.float32)

print("[INFO] ZED camera opened successfully")
print("[INFO] Intrinsics loaded from calibration parameters")

# --- ArUco Setup ---
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
detectorParams = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, detectorParams)

# --- Prepare Image Containers ---
left_image = sl.Mat()
right_image = sl.Mat()
runtime = sl.RuntimeParameters()

print("[INFO] Starting ZED video stream...")
time.sleep(1.0)

# --- Main Loop ---
while True:
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        # Retrieve both images
        zed.retrieve_image(left_image, sl.VIEW.LEFT)
        zed.retrieve_image(right_image, sl.VIEW.RIGHT)

        left_frame = left_image.get_data()
        right_frame = right_image.get_data()
        left_frame = cv.cvtColor(left_frame, cv.COLOR_RGBA2BGR)
        right_frame = cv.cvtColor(right_frame, cv.COLOR_RGBA2BGR)

        # Run ArUco detection on left frame
        left_frame = pose_estimation(left_frame, detector, cameraMatrix, distortion)

        # Combine left + right for display
        combined = np.hstack((left_frame, right_frame))
        combined = imutils.resize(combined, width=1500)

        cv.imshow("ZED Left + Right (ArUco on Left)", combined)

        # Quit with 'q'
        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            break

# --- Cleanup ---
cv.destroyAllWindows()
zed.close()
print("[INFO] ZED camera closed.")
