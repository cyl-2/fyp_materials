#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

"""
This script will handle the flight planning and control sequences for delivering a payload 
to a designated location and returning to the home base after delivery.
"""

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for the drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    
    # Fetch and print vehicle state (battery, GPS, etc.)
    asyncio.ensure_future(print_battery(drone))
    asyncio.ensure_future(print_gps_info(drone))
    asyncio.ensure_future(print_in_air(drone))
    asyncio.ensure_future(print_position(drone))

    # System initialization and takeoff
    print("Arming")
    await drone.action.arm()
    print("Taking off")
    await drone.action.takeoff()

    # Ascend to a predetermined altitude and set airspeed
    flying_altitude = 15.0  # meters
    delivering_altitude = 0.3  # meters
    target_airspeed = 12.0  # meters per second
    await drone.action.set_maximum_speed(target_airspeed)

    # Define mission waypoints
    # 1st waypoint: a waypoint near the delivering area
    # 2nd waypoint: the actual delivery area
    # 3rd waypoint: a waypoint used to start ascending back to the target altitud
    mission_items = []
    mission_items.append(MissionItem(47.4853513,
                                     11.2348653,
                                     flying_altitude,
                                     target_airspeed,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.VehicleAction.NONE))
    mission_items.append(MissionItem(48.3696513,
                                     8.68431,
                                     delivering_altitude,
                                     target_airspeed,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.VehicleAction.NONE))
    mission_items.append(MissionItem(49.3696513,
                                     8.68431,
                                     flying_altitude,
                                     target_airspeed,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.VehicleAction.NONE))                                 
    mission_plan = MissionPlan(mission_items)

    # Upload and start mission
    print("Uploading mission")
    await drone.mission.set_return_to_launch_after_mission(True) # once all waypoints are visited, return to home base
    await drone.mission.upload_mission(mission_plan)

    print("Starting mission")
    await drone.mission.start_mission()

    # Wait until delivery point is reached to simulate payload release
    print("Flying to delivery point...")
    async for mission_progress in drone.mission.mission_progress():
        if mission_progress.current == 2:  # Delivery point is the 2nd waypoint
            print("Payload being released...")
            
            await drone.gripper.release(instance=1)
            
            await asyncio.sleep(5)  # Simulate time for payload release
            print("Payload delivered successfully.")
            break

    print("Returning to home base")
    async for mission_progress in drone.mission.mission_progress():
        if mission_progress.current == len(mission_plan.mission_items):
            print("Mission completed")
            break

    # Landing
    print("Initiating landing")
    await drone.action.land()


async def print_battery(drone):
    async for battery in drone.telemetry.battery():
        print(f"Battery: {battery.remaining_percent}")


async def print_gps_info(drone):
    async for gps_info in drone.telemetry.gps_info():
        print(f"GPS info: {gps_info}")


async def print_in_air(drone):
    async for in_air in drone.telemetry.in_air():
        print(f"In air: {in_air}")


async def print_position(drone):
    async for position in drone.telemetry.position():
        print(position)

if __name__ == "__main__":
    asyncio.run(run())
