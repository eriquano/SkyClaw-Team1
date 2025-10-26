# # import pyzed.sl as sl

# # def main():
# #     print("🔍 ZED Diagnostic Test (Python)")
# #     print("=================================\n")

# #     zed = sl.Camera()

# #     init_params = sl.InitParameters()
# #     init_params.camera_resolution = sl.RESOLUTION.HD720
# #     init_params.camera_fps = 30
# #     init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
# #     init_params.coordinate_units = sl.UNIT.METER
# #     init_params.sdk_verbose = 1

# #     print("➡️ Opening ZED camera...")
# #     err = zed.open(init_params)

# #     if err != sl.ERROR_CODE.SUCCESS:
# #         print(f"❌ Failed to open ZED camera: {err.name}")
# #         return

# #     print("✅ ZED camera opened successfully!\n")

# #     info = zed.get_camera_information()
# #     print("\n--- Camera Information ---")
# #     print(f"Serial Number: {info.serial_number}")
# #     print(f"Firmware: {info.camera_configuration.firmware_version}")
# #     print(f"Model: {info.camera_model}")

# #     try:
# #         res = info.camera_configuration.resolution
# #         print(f"Configured resolution: {res.width}x{res.height}")
# #     except AttributeError:
# #         print("Configured resolution: unknown (attribute not present)")

# #     try:
# #         calib = info.calibration_parameters.left_cam
# #         print(f"Focal length (fx, fy): {calib.fx}, {calib.fy}")
# #     except AttributeError:
# #         print("Calibration: not available (or inaccessible attribute)")

# #     sensors = sl.SensorsData()
# #     err_sens = zed.get_sensors_data(sensors, sl.TIME_REFERENCE.CURRENT)

# #     print("\n--- Sensor Data ---")
# #     if err_sens == sl.ERROR_CODE.SUCCESS:
# #         imu_data = sensors.get_imu_data()
# #         print("✅ IMU data successfully read.")
# #         print(f"Angular velocity: {imu_data.get_angular_velocity()}")
# #         print(f"Linear acceleration: {imu_data.get_linear_acceleration()}")
# #     elif err_sens in [sl.ERROR_CODE.MOTION_SENSORS_REQUIRED, sl.ERROR_CODE.SENSORS_NOT_AVAILABLE]:
# #         print("⚠️ Failed to read IMU / sensors data: Sensors interface not available in WSL.")
# #     else:
# #         print(f"⚠️ Failed to read IMU / sensors data: {err_sens.name}")

# #     zed.close()
# #     print("\n🔒 ZED camera closed cleanly.")

# # if __name__ == "__main__":
# #     main()


# import cv2
# import numpy as np

# # Adjust based on your actual resolution
# FRAME_WIDTH = 2560
# FRAME_HEIGHT = 720

# cap = cv2.VideoCapture("/dev/video0")

# if not cap.isOpened():
#     print("❌ Failed to open /dev/video0")
#     exit(1)

# print("✅ ZED camera stream opened")

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("⚠️ Failed to grab frame")
#         break

#     # Split into left and right images
#     left = frame[:, :FRAME_WIDTH // 2]
#     right = frame[:, FRAME_WIDTH // 2:]

#     # Run your Aruco detection here (example placeholder)
#     # gray_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
#     # corners, ids, _ = aruco_detector.detectMarkers(gray_left)
#     # left = cv2.aruco.drawDetectedMarkers(left, corners, ids)

#     combined = np.hstack((left, right))
#     cv2.imshow("ZED2 Left | Right", combined)

#     key = cv2.waitKey(1)
#     if key == 27:  # ESC to exit
#         break

# cap.release()
# cv2.destroyAllWindows()

import cv2
print(cv2.__version__)
