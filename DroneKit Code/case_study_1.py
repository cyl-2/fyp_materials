from dronekit import connect, VehicleMode, LocationGlobal
import time
import argparse

# Get some vehicle attributes
def get_vehicle_info():
    print("\nGet all vehicle attribute values:")
    print(" Autopilot Firmware version: %s" % vehicle.version)
    print(" Global Location: %s" % vehicle.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print(" Local Location: %s" % vehicle.location.local_frame)
    print(" Attitude: %s" % vehicle.attitude)
    print(" Velocity: %s" % vehicle.velocity)
    print(" GPS: %s" % vehicle.gps_0)
    print(" Gimbal status: %s" % vehicle.gimbal)
    print(" Battery: %s" % vehicle.battery)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
    print(" Airspeed: %s" % vehicle.airspeed)    # settable
    print(" Mode: %s" % vehicle.mode.name)    # settable
    print(" Armed: %s" % vehicle.armed)    # settable

# Arming the vehicle
def arm():
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    #print(vehicle.mode)
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    
    #print("Armed or not: ", vehicle.armed)
    print("Armed!")

# Preparing the vehicle for takeoff
def takeoff(target_altitude):
    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)  # Takeoff to target altitude

    # Wait until the vehicle reaches the target altitude before starting the mission
    while not vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)
        # If correct altitude reached, start the mission
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            vehicle.airspeed = 5
            
def goto_waypoint_and_land(target_location):
    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")

    print("FLying to the target waypoint!")
    vehicle.simple_goto(LocationGlobal(target_location))
    # There is no built-in function that notifies the user when the location has been reached
    # Once the location has been reached, the next line of code gets executed immediately

    print("Landing the vehicle")
    vehicle.mode = VehicleMode("LAND")


if __name__ == "__main__":

    # Set up option parsing to get connection string
    
    parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
    parser.add_argument('--connect',
                        help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None

    # Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the vehicle
    vehicle = connect(connection_string, wait_ready=True)
    #vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    get_vehicle_info()

    target_location = [-28.456457, 168.5434534, 20] #dest_latitude, dest_longitude, dest_altitude]

    print('Launching Drone...')
    
    arm()
    takeoff(target_location[2])
    goto_waypoint_and_land(target_location)
    
    vehicle.close()
    if sitl:
        sitl.stop()