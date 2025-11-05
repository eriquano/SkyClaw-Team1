import cv2

PORT = 5000

# GStreamer pipeline to receive video
gst_in = (
    f'udpsrc port={PORT} caps="application/x-rtp, encoding-name=H264, payload=96" ! '
    f'rtph264depay ! avdec_h264 ! videoconvert ! appsink'
)

cap = cv2.VideoCapture(gst_in, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Cannot open stream")
    exit()

print(f"Listening for stream on port {PORT}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Stream ended or no data")
        break

    cv2.imshow("Jetson Stream", frame)
    if cv2.waitKey(1) == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
