import cv2
import imagezmq

receiver_ip = ""  # <--- put Machine A's Tailscale IP here
sender = imagezmq.ImageSender(connect_to=f"tcp://{receiver_ip}:5555")

