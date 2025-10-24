from mavsdk import System
import asyncio
from Video import Video
import cv2
from datetime import datetime

class Drone:
    def __init__(self):
        self.drone = System()

    async def takeoff(self):
        print("--Taking off")
        await self.drone.action.takeoff()
        
        await asyncio.sleep(10)


    async def arm(self):
        await self.drone.connect(system_address="udp://:14540")

        status_text_task = asyncio.ensure_future(Drone.print_status_text(self.drone))

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

        print("-- Arming")
        await self.drone.action.arm()

        print("Drone ready to fly")

    @staticmethod
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

async def update_frame(video_source, port):
    if video_source.frame_available():
        frame = video_source.frame()

        # maybe make it stream somewhere else if it is not connected to localhost (see examples)
        cv2.imshow(f'Camera Feed {port}', frame)

async def run_green(drone, video_source):
    while True: 
        if video_source.frame_available():
            frame = video_source.frame()
            frame = cv2.resize(frame, (1280, 720))

async def main():
    drone = Drone()

    #arm drone -- change to not make it arm and only connect
    await drone.arm()
    # temp for testing cameras
    await drone.takeoff()
    
    # creates a Video object for each camera with right UDP ports
    video_source_down = Video(port=5600)
    video_source_forward = Video(port=5601)

    try: 
        print("Beginning main loop")

        while True: 
            # update_frame is purely to show the user the video
            # run_green will takeoff, find green object, go pick it up, do idk what with it, land again?
            await update_frame(video_source_down, 5600)
            await update_frame(video_source_forward, 5601)
            #await run_green()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exiting")
                break
    except KeyboardInterrupt: 
        print("Force shutting down.")
    
    finally:
        print("--Closing all windows")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    asyncio.run(main())