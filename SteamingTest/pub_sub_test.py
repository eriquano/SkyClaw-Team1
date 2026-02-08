"""pub_sub_broadcast.py -- broadcast OpenCV stream using PUB SUB."""

import sys
import socket
import traceback
from time import sleep
import cv2
from imutils.video import VideoStream
import imagezmq
import numpy as np  # Added for array operations
import cv2.aruco as aruco  # Added for ArUco detection
import time  # Added for FPS calculation

# --- START: Added from go_green_simple.py ---

# 1. ArUco Configuration
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
MARKER_LENGTH = 0.1  # in meters. TODO: REPLACE WITH ACTUAL MARKER SIZE

# 2. Calibrate Camera
# TODO: REPLACE WITH ACTUAL CAMERA CALIBRATION CONSTANTS!!!!
camera_matrix = np.array([[600, 0, 320],
                          [0, 600, 240],
                          [0,   0,   1]], dtype=np.float32)

dist_coeffs = np.zeros((5, 1))  # Replace if using lens distortion

# 3. Green Detection
green_lower = np.array([36, 100, 100])
green_upper = np.array([86, 255, 255])
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

# --- END: Added from go_green_simple.py ---


if __name__ == "__main__":
    # Publish on port
    port = 5000
    sender = imagezmq.ImageSender("tcp://*:{}".format(port), REQ_REP=False)

    # Open input stream
    capture = VideoStream(src=0)  # Using src=0 for a standard webcam
    capture.start()
    sleep(2.0)  # Warmup time
    print("Input stream opened")

    # JPEG quality, 0 - 100
    jpeg_quality = 50
    rpi_name = socket.gethostname()

    # --- ADDED: Variables for text display ---
    prev_time = 0  # For FPS calculation
    
    try:
        counter = 0
        while True:
            frame = capture.read()
            if frame is None:
                break

            # --- ADDED: FPS Calculation ---
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time
            fps_text = f"FPS: {fps:.2f}"

            # --- ADDED: Initialize status text for this frame ---
            green_status_text = "Green: Searching..."
            green_pos_text = "Green Pos: N/A"
            aruco_text = "ArUco: Searching..."
                
            # --- START: CV Logic from go_green_simple.py ---

            # 1. Green Cube Detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, green_lower, green_upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                c = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    frame, "Green Cube", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )
                
                # --- ADDED: Update green text ---
                cx = x + w // 2
                cy = y + h // 2
                green_status_text = "Green: DETECTED"
                green_pos_text = f"Green Pos: ({cx}, {cy})"

            # 2. ArUco Detection & Pose Estimation
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 11, 2
            )
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT)

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_LENGTH, camera_matrix, dist_coeffs
                )
                
                cv2.aruco.drawDetectedMarkers(frame, corners)
                for i in range(len(ids)):
                    cv2.drawFrameAxes(
                        frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1
                    )

                # --- ADDED: Update ArUco text (shows first marker) ---
                first_id = ids.flatten()[0]
                first_tvec = tvecs[0][0]
                tvec_str = f"[{first_tvec[0]:.2f}, {first_tvec[1]:.2f}, {first_tvec[2]:.2f}]"
                aruco_text = f"ArUco ID {first_id}: Tvec {tvec_str}"
            
            # --- END: CV Logic ---

            # --- ADDED: Draw all text on frame ---
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            text_thickness = 2
            
            # Colors
            white = (255, 255, 255)
            green = (0, 255, 0)
            yellow = (0, 255, 255)

            # Draw FPS (White)
            cv2.putText(frame, fps_text, (10, 30), font, font_scale, white, text_thickness)
            
            # Draw Green Status (Green if found, else White)
            g_color = green if "DETECTED" in green_status_text else white
            cv2.putText(frame, green_status_text, (10, 60), font, font_scale, g_color, text_thickness)
            
            # Draw Green Position (White)
            cv2.putText(frame, green_pos_text, (10, 90), font, font_scale, white, text_thickness)

            # Draw ArUco Status (Yellow if found, else White)
            a_color = yellow if "ID" in aruco_text else white
            cv2.putText(frame, aruco_text, (10, 120), font, font_scale, a_color, text_thickness)
            # --- END: Text drawing ---

            # Encode and send the *modified* frame
            ret_code, jpg_buffer = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
            )
            if ret_code:
                sender.send_jpg(rpi_name, jpg_buffer)

            counter = counter + 1

    except (KeyboardInterrupt, SystemExit):
        print('Exit due to keyboard interrupt')
    except Exception as ex:
        print('Python error with no Exception handler:')
        print('Traceback error:', ex)
        traceback.print_exc()
    finally:
        capture.stop()
        sender.close()
        sys.exit()