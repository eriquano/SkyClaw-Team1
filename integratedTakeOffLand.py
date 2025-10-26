#!/usr/bin/env python3

import asyncio
from mavsdk import System

import cv2 as cv
import cv2.aruco as aruco
import numpy as np
from imutils.video import VideoStream
import time
import imutils

#function to estimate pose of aruco marker
def pose_estimation(frame, aruco_dict, parameters, matrix_coefficients, distortion_coefficients):
    # Detect ArUco markers in the frame
    corners, ids, rejected = cv.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each detected marker
            rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(
                corners[i], 0.025, matrix_coefficients, distortion_coefficients
            )

            # Draw detected marker boundaries and axes
            cv.aruco.drawDetectedMarkers(frame, corners)
            cv.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
    
    # Return the annotated frame
    return frame

def open_zed():
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.METER
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("Could not open ZED camera")

    info = zed.get_camera_information()
    left = info.camera_configuration.calibration_parameters.left_cam
    K = np.array([[left.fx, 0, left.cx],
                  [0, left.fy, left.cy],
                  [0, 0, 1]], dtype=np.float32)
    D = np.array(left.disto, dtype=np.float32)
    print("[INFO] ZED opened; intrinsics loaded.")
    return zed, K, D, sl.RuntimeParameters()

def close_zed(zed):
    zed.close()
    print("[INFO] ZED closed.")



#camera + marker setup
#dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
#parameters = aruco.DetectorParameters()

# Load your camera calibration (intrinsic matrix and distortion coefficients)
#cameraMatrix = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
#distortion = np.array((-0.43948, 0.18514, 0, 0)) 
#data = np.load("webcam_chessboard_calib_1280x720.npz")
#cameraMatrix = data["K"]
#distortion = data["dist"]

def make_detector():
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    params = aruco.DetectorParameters()
    if hasattr(aruco, "ArucoDetector"):
        return aruco.ArucoDetector(dictionary, params)
    # legacy fallback
    class Legacy:
        def __init__(self,d,p): self.d,self.p=d,p
        def detectMarkers(self,frame):
            gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
            return aruco.detectMarkers(gray,self.d,parameters=self.p)
    return Legacy(dictionary,params)


# scan for marker 
def scan_for_marker(timeout_s=15, preview=False):
    """
    Continuously grab frames from the webcam, use your pose_estimation()
    to check for ArUco markers, and return True if one is found before timeout.
    """
    vs = VideoStream(src=0).start()
    time.sleep(2.0)  # allow camera to warm up

    t0 = time.time()
    seen = False

    try:
        while (time.time() - t0) < timeout_s:
            frame = vs.read()
            if frame is None:
                continue

            frame = imutils.resize(frame, width=1000)

            # Run your pose estimation
            output = pose_estimation(frame, dictionary, parameters, cameraMatrix, distortion)

            # If any markers are visible, mark as seen
            corners, ids, _ = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            if ids is not None and len(ids) > 0:
                seen = True

            if preview:
                cv.imshow("ArUco Detection", output)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break

            if seen:
                break

    finally:
        vs.stop()
        if preview:
            cv.destroyAllWindows()

    return seen

async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"[STATUS] {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

#wait for marker
    print("Scanning for ArUco marker before takeoff...")
    seen = await asyncio.to_thread(scan_for_marker, 20, True)
    if not seen:
        print("Marker not detected — aborting mission.")
        status_text_task.cancel()
        return        

 
    print("-- Arming")
    await drone.action.arm()


    print("-- Taking off")
    await drone.action.set_takeoff_altitude(3.0)
    await drone.action.takeoff()
    await asyncio.sleep(10)


# --- AFTER TAKEOFF: SCAN AGAIN TO LAND ---
    print("Scanning for ArUco marker before landing...")
    seen2 = await asyncio.to_thread(scan_for_marker, 25, True)
    if seen2:
        print("-- Marker detected again, landing...")
        await drone.action.land()
    else:
        print("Marker not found — fallback landing.")
        await asyncio.sleep(5)
        await drone.action.land()



    
    await asyncio.sleep(5)
    status_text_task.cancel()


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
