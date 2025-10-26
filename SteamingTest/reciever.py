import cv2
import imagezmq

# Listen on port 5555 on all interfaces
image_hub = imagezmq.ImageHub(open_port="tcp://*:5555")

print("Receiver is waiting for images...")


cam_name, jpg_buffer = image_hub.recv_jpg()
image_hub.send_reply(b"OK")  # required handshake

frame = cv2.imdecode(jpg_buffer, cv2.IMREAD_COLOR)
cv2.imshow(cam_name, frame)

cv2.waitKey(0)
