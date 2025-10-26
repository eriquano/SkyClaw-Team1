from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw
import asyncio
from Video import Video
import cv2
import numpy as np

class Drone:
    def __init__(self):
        self.drone = System()

    async def takeoff(self):
        print("Taking off")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        
        await asyncio.sleep(4)
        print("-- Takeoff finished")


    async def arm(self):
        print("-- Arming")
        await self.drone.action.arm()

        print("-- Setting initial setpoint")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
            print("Drone ready to offboard")
        except OffboardError as error:
            print(
                f"Starting offboard mode failed with error code: \
                {error._result.result}"
            )
            print("-- Disarming")
            await self.drone.action.disarm()
            return

    async def connect(self):
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")

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

        print("Drone ready to arm")

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
    # cv2.destroyWindow('Camera Feed 5600')
    # cv2.destroyWindow('Camera Feed 5601')
    cv2.destroyAllWindows()

    #do we assume it has already taken off or what idk?
    await drone.takeoff()


    # just putting these here for now
    green_lower = np.array([36, 100, 100])
    green_upper = np.array([86, 255, 255])
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))

    while True: 
        # should we count if frames aren't avaliable for a long time then land or something like that?
        if video_source.frame_available():
            frame = video_source.frame()
            frame = cv2.resize(frame, (1280, 720))

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, green_lower, green_upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # shows the green objects on screen
            # for c in contours:
            #     x, y, w, h = cv2.boundingRect(c)
            #     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #     cv2.putText(frame, "Green Object", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            # cv2.imshow("All Green Objects", frame)

            for c in contours:
                # how should we determine which green object to grab, closest, largest, smallest
                # for know I will use closest
                
                # we could copy the green tracker code from here now?

                pass

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("-- Exiting run_green")

                #putting this here for now because i dont know what windows if any there are
                cv2.destroyAllWindows()
                break
        else:
            print("frame not avaliable")

def distance_to_center(c, frame_center_x, frame_center_y):
        x, y, w, h = cv2.boundingRect(c)
        cx = x + w // 2
        cy = y + h // 2
        return np.sqrt((cx - frame_center_x)**2 + (cy - frame_center_y)**2)

async def main():
    drone = Drone()

    await drone.connect()

    await drone.arm()
    
    # creates a Video object for each camera with right UDP ports
    video_source_down = Video(port=5600)
    video_source_forward = Video(port=5601)

    try: 
        print("Beginning main loop")

        while True: 
            # update_frame is purely to show the user the video
            # run_green will takeoff, find green object, go pick it up, do idk what with it, land again?
            await update_frame(video_source_down, 5600)
            # await update_frame(video_source_forward, 5601)

            # IDK which one to use
            #await run_green()
            # asyncio.create_task(run_green())
            if cv2.waitKey(1) & 0xFF == ord('g'):
                print("Starting green search")
                await run_green(drone, video_source_down)

                
            elif cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exiting")
                break

    except KeyboardInterrupt: 
        print("Force shutting down.")

    finally:
        print("--Closing all windows")
        cv2.destroyAllWindows()

        # temporary place for this // add to drone class later
        print("-- Stopping offboard")
        try:
            await drone.drone.offboard.stop()
        except OffboardError as error:
            print(
                f"Stopping offboard mode failed with error code: \
                {error._result.result}"
            )
        
        await drone.drone.action.land()
        await drone.drone.action.disarm()


if __name__ == '__main__':
    asyncio.run(main())