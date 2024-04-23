#!/usr/bin/env python
# -*- coding: utf-8 -*-


from dronekit import connect, VehicleMode, LocationGlobal
import time
import argparse

class Drone:
    def __init__(self, vehicle, target_location):
        # Connect to the Vehicle
        self._log('Connected to vehicle.')

        """
        Initializes the Drone object.
        
        Args:
            vehicle (Vehicle): The connected vehicle object.
            target_location (list): [latitude, longitude, altitude] of target location.
        """

        self.vehicle = vehicle
        self.home_location = [self.vehicle.location.global_frame.lat,
                            self.vehicle.location.global_frame.lon, 60.0]
        self.target_location = target_location
        
        self.navigation_system = NavigationSystem(self.vehicle)
        self.payload_delivery_system = PayloadDeliverySystem(self.vehicle)

        self._log("Drone preparing for launch")

        # Register observers
        self.vehicle.add_attribute_listener('location',  self.navigation_system.location_callback)

    def launch(self):
        self._log('Launching drone...')
        
        self._log("Waiting to arm...")
        while not self.vehicle.is_armable:
            time.sleep(.1)

        retries = 3
        for i in range(retries):
            self._log("Attempt at launching")
            print(self.vehicle.mode)
            self.vehicle.mode = VehicleMode("GUIDED")
            #print(self.vehicle.mode)
            self.arm()
            self.takeoff()

    def arm(self, value=True): 
        if value:
            self.vehicle.armed = True
            while not self.vehicle.armed:
                time.sleep(.1)
        else:
            self.vehicle.armed = False

    def takeoff(self):
        self._log("Taking off")

        self.vehicle.airspeed = 5 # 5 meters per second
        self.vehicle.simple_takeoff(self.target_location[2])  

        # Wait for takeoff to complete
        # Creates a loop that continues until the drone's current altitude >= 95% of the desired altitude
        altitude = self.target_location[2]
        while not self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Altitude: ", self.vehicle.location.global_relative_frame.alt)
            time.sleep(1)

        # If correct altitude reached, start the mission
        if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            self.start_mission()
        else:
            # Autonomous fail-safe landing procedure
            self._log("Vehicle failed launch - activated fail-safe landing procedure")
            self.vehicle.mode = VehicleMode("LAND") 
            while self.vehicle.armed:
                time.sleep(1)

    def start_mission(self):
        self._log("Mission START")
        # Set mode to AUTO to start mission
        self.vehicle.mode = VehicleMode("AUTO")
        self._log("Start navigation to target location")
        self.navigation_system.navigate(self.target_location)
        self.payload_delivery_system.payload_delivery()
  
        self._log("Returning to home base")
        self.navigation_system.navigate(self.home_location, True)

    def _log(self, message):
        print("[DEBUG]: {0}".format(message))

class NavigationSystem:

    """
    Navigation operations
    """

    def __init__(self, vehicle):
        self.vehicle = vehicle

    # Navigates the drone to a given location.
    def navigate(self, location, land=False):
        # Plan route and navigate
        self.vehicle.simple_goto(LocationGlobal(location)) # coordinates are absolute positions on the Earth's surface

        # Wait for navigation to complete
        while not self.location_reached():
            if self.obstacles_detected():
                # TODO: Attempt route replanning
                # TODO: If replanning fails, activate fail-safe route
                # TODO: Notify operator for manual intervention if necessary
                break

            if self.harsh_weather_conditions():
                # TODO:Adjust flight parameters for stability
                # TODO: Consider rerouting if necessary
                break
            time.sleep(1) 

        # Land the UAV if this is navigating back home
        if land is True:
            self.vehicle.mode = VehicleMode("RTL")
            while self.vehicle.armed:
                time.sleep(1)
            # TODO: Implement proper logic for landing

        
    # Checks if the drone has reached the target location.
    def location_reached(self):
        # TODO: Implement your logic to check if the drone is at the target location
        target = [self.vehicle.target_location[0], self.vehicle.target_location[1]]
        current_location = [self.vehicle.current_location.lat, self.vehicle.current_location.lon]
        if current_location == target:
            return True

    # Receive updates about the vehicle's location
    def location_callback(self, vehicle, name, location):
        if self.vehicle.location.global_relative_frame.alt is not None:
            self.vehicle.altitude = self.vehicle.location.global_relative_frame.alt

        self.vehicle.current_location = self.vehicle.location.global_relative_frame

class PayloadDeliverySystem:
    """
    Managing payload delivery operations

    Args:
        vehicle (Vehicle): The connected vehicle object.
        delivering_altitude (float): The altitude at which payload delivery should occur.
    """

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.delivering_altitude = 30.0

    # Deploying the payload
    def payload_delivery(self):
        # Verify UAV is at the correct altitude
        self.adjust_altitude(delivering_altitude)
        if self.vehicle.location.global_relative_frame.alt == delivering_altitude:
            # Deploy payload - code taken from https://stackoverflow.com/questions/36636073/is-it-possible-to-control-an-actuator-using-dronekit-python
            msg = vehicle.message_factory.command_long_encode(
            0, 0,    # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
            0, #confirmation
            1,    # servo number
            1500,          # servo position between 1000 and 2000
            0, 0, 0, 0, 0)    # param 3 ~ 7 not used

            # send command to vehicle
            vehicle.send_mavlink(msg)
            # TODO: Check payload deployment status
            pass

    # Altitude adjustment at target location
    def adjust_altitude(self, altitude):
        self.vehicle.simple_goto(LocationGlobal(self.vehicle.target_location[0], self.vehicle.target_location[1], delivering_altitude))

        # Wait for altitude adjustment to complete
        while not abs(self.vehicle.location.global_relative_frame.alt - target_altitude) < 0.5:
            if self.altitude_adjustment_fails():
                # TODO: Ascend to a safe altitude
                # TODO: Notify operator for manual intervention
                break
            time.sleep(1)
    
    def altitude_adjustment_fails(self):
        # TODO: Method to check if the altitude adjustment was successful or not
        pass



if __name__ == "__main__":

    # Set up option parsing to get connection string
    
    parser = argparse.ArgumentParser(description='Commands vehicle')
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

    target_location = [-35.361354, 149.165218, 20] #dest_latitude, dest_longitude, dest_altitude]

    print('Launching Drone...')
    Drone(vehicle, target_location).launch()

    vehicle.close()