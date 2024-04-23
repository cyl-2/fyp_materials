#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

async def run():
    # Initialize the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    
    # Print static vehicle information
    info = await drone.info.get_version() # firmware info
    print(info)
    info2 = await drone.info.get_flight_information() # flight info
    print(info2)

    print("Waiting for global position estimate")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate found")
            break

    # Arm the vehicle and takeoff
    print("Arming")
    await drone.action.arm()
    print("Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(5)

    # Set airspeed
    target_airspeed = 5.0  # m/s
    await drone.action.set_maximum_speed(target_airspeed)

    # Send the drone to the destined waypoint
    target_altitude = 10.0  # meters
    await drone.action.goto_location(47.398039859999997,
                                      8.5455725400000002,
                                      target_altitude, 0)

    # Landing
    print("Initiating landing")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())
