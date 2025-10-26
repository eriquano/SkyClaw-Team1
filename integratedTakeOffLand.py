#!/usr/bin/env python3

import asyncio
from mavsdk import System


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

##initialize aruco
#dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250);
#detectorParams = aruco.DetectorParameters();
#cameraMatrix = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
#distortion = np.array((-0.43948, 0.18514, 0, 0))
 
    print("-- Arming")
    await drone.action.arm()

# enter a loop to detect aruco marker
# capture a frame from camera
# detect aruco marker by calling
# corners, ids, rejected = cv.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
# check if ids has the right aruco marker and takeoff

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

# enter a loop to detect aruco marker
# capture a frame from camera
# detect aruco marker by calling
# corners, ids, rejected = cv.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
# check if ids has the right aruco marker and land


    print("-- Landing")
    await drone.action.land()

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