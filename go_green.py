import cv2
import numpy as np
import argparse
import math
import socket, struct
from collections import deque
import mavsdk
# import rclpy
import asyncio
from Video import Video
# from rclpy.node import Node
# from geometry_msgs.msg import Vector3Stamped


async def connect():

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

    print("-- Arming")
    await drone.action.arm()

# currently not working
# also want to add front-facing camera
async def camera():
    """
    Adds downward facing camera in Gazebo Classic

    Parameters
    ---------
    None

    Returns
    -------
    None
    """
    # await drone.action.arm()
    # await drone.action.takeoff()
    video_source = Video(port=5601)

    while True:
        if video_source.frame_available:
            frame = video_source.frame()

            cvs.imshow(f"Drone Camera", frame)
            cvs.waitKey(1)

# look for stuff in TAR github.. maybe MAVtesting

async def print_status_text(drone):
    """
    Check status of drone

    Used in asyncio.connect()
    """
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return

# take off
# how high go?
async def takeoff():
    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

# run green tracker
# import greenTracker.py
# greenTracker.function()

# if see green, hover
# how long hover?
# how high hover?
# how can it see the green? do i need to add camera?
# will it know if sees green outside aruco markers?
# go back to land pad?

# land
async def land():
    print("-- Landing")
    await drone.action.land()

    status_text_task.cancel()

def main():
    asyncio.connect()
    asyncio.camera()
    asyncio.takeoff()
    # asyncio.find green and go
    # asyncio go down and back up
    # return to place
    asyncio.land()

if __name__ == '__main__':
    main()