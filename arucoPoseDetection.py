import cv2 as cv
import cv2.aruco as aruco
import numpy as np
from imutils.video import VideoStream
import time
import imutils

#Use following to install imutils for Python 3: pip3 install imutils


def pose_estimation(frame, aruco_dict, parameters, matrix_coefficients, distortion_coefficients):

    corners, ids, rejected = cv.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if len(corners) > 0:
        for i in range(0, len(ids)):
        
            rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners[i], 0.025, matrix_coefficients, distortion_coefficients)
            
            cv.aruco.drawDetectedMarkers(frame, corners) 
            
            #printing rvec and tvec on the frame if we need this as an output I can add that to the return values
            cv.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01) 
         
    return frame

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250);
detectorParams = aruco.DetectorParameters();


#cameraMatrix = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
#distortion = np.array((-0.43948, 0.18514, 0, 0)) 

# webcam_chessboard_calib_1280x720.npz is my laptop camera calibration. If that does not work you will have to
# calibrate your own or try the camera matrix commented above
data = np.load("webcam_chessboard_calib_1280x720.npz")  
cameraMatrix  = data["K"]
distortion = data["dist"]

print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)


# loop over the frames from the video stream
while True:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 1000 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=1000)


	frame = pose_estimation(frame, dictionary, detectorParams, cameraMatrix, distortion)

	# show the output frame
	cv.imshow("Frame", frame)
	key = cv.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
# do a bit of cleanup
cv.destroyAllWindows()
vs.stop()
