import cv2
import imagezmq
import numpy as np

receiver_ip = "100.86.186.99"  # <--- put Machine A's Tailscale IP here
sender = imagezmq.ImageSender(connect_to=f"tcp://{receiver_ip}:5555")

cap = cv2.imread("goat.png",  cv2.IMREAD_COLOR)

# Convert to HSV (better for color detection than BGR)
hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)

# Define a "blue" color range in HSV
lower_blue = np.array([100, 120, 50])
upper_blue = np.array([140, 255, 255])

# Create a mask where blue is white and everything else is black
mask = cv2.inRange(hsv, lower_blue, upper_blue)

# Find contours in the mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

if contours:
    # Pick the largest blue contour
    largest = max(contours, key=cv2.contourArea)

    # Get a circle that encloses it
    (x, y), radius = cv2.minEnclosingCircle(largest)
    center = (int(x), int(y))
    radius = int(radius)

    # Draw the circle and center point on the original image
    cv2.circle(cap, center, radius, (0, 255, 0), 3)      # green circle
    cv2.circle(cap, center, 3, (0, 0, 255), -1)          # red center dot

ret, jpg = cv2.imencode(".jpg", cap, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
sender.send_jpg("goat", jpg)