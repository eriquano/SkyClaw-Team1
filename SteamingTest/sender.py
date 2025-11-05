import cv2

LAPTOP_IP = "192.168.1.10" #use Ethan's laptop IP or Sima's laptop for now
PORT = 5000

# GStreamer pipeline to send video
gst_out = (
    f'appsrc ! videoconvert ! '
    f'nvv4l2h264enc bitrate=4000000 insert-sps-pps=true ! '
    f'rtph264pay config-interval=1 pt=96 ! '
    f'udpsink host={LAPTOP_IP} port={PORT}'
)

# Open camera
cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Set resolution / FPS
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

# Open output stream
out = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, 30, (1280, 720))
if not out.isOpened():
    print("Cannot open output pipeline")
    exit()

print(f"Streaming to {LAPTOP_IP}:{PORT}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame capture failed")
        break

    # Example processing
    # cv2.putText(frame, "Jetson Stream", (30, 60),
    #             cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)

    out.write(frame)

cap.release()
out.release()
print("Stream ended")
