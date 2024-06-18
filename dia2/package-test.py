import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative
import cv2
import argparse
import math
import time
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import numpy as np
from cv2 import aruco
from pymavlink import mavutil
# import RPi.GPIO as GPIO

class Vehicle():

    def __init__(self):

        self.state = "Initial"
        self.current_wp = 0
        self.current_lat = 0
        self.current_lon = 0
        self.current_alt = 0
        self.vehicle_mode = ''
        self.started = False
        self.count = 0
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default = 'tcp:127.0.0.1:5763')
        args = parser.parse_args()

        # self.electromagnet = 11
        # GPIO.setmode(GPIO.BOARD)
        # GPIO.setup(self.electromagnet, GPIO.OUT)
        # GPIO.output(self.electromagnet, True)
        self.delivered = False

        #-- Connect to the vehicle
        print('Connecting...')
        self.vehicle = connect(args.connect)
        self.cmds = self.vehicle.commands
        self.cmds.download()
        self.cmds.wait_ready()

        #-- Check vehicle status
        print(f"Mode: {self.vehicle.mode.name}")
        print(" Global Location: %s" % self.vehicle.location.global_frame)
        print(" Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)
        print(" Local Location: %s" % self.vehicle.location.local_frame)
        print(" Attitude: %s" % self.vehicle.attitude)
        print(" Velocity: %s" % self.vehicle.velocity)
        print(" Gimbal status: %s" % self.vehicle.gimbal)
        print(" EKF OK?: %s" % self.vehicle.ekf_ok)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Rangefinder: %s" % self.vehicle.rangefinder)
        print(" Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        print(" Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Armed: %s" % self.vehicle.armed)    # settable

        @self.vehicle.on_message('GLOBAL_POSITION_INT')
        def listener(_, name, message):
            self.current_lat = message.lat
            self.current_lon = message.lon
            self.current_alt = message.relative_alt / 1000.0
        
        @self.vehicle.on_message('MISSION_CURRENT')
        def waypoint_listener(_, name, message):
            # print("Reached waypoint %d" % message.seq)
            self.current_wp = message.seq

    def run(self):
            # Check current state and execute corresponding logic
            while True:
                # print("Run")
                if self.state == "Initial":
                    # print("Go to Initial")
                    self.delivery_state_logic()
                elif self.state == "PrecisionLanding":
                    self.precision_landing_state_logic()
                elif self.state == "Landed":
                    if self.landed_state_logic():
                        break
            # Add more states and transitions as needed
    
    def delivery_state_logic(self):

        dist = self.distance_to_current_waypoint()
        # print(dist)
    
        if(self.vehicle.commands.next == 2):
                self.count = self.count + 1
                if self.delivered == False and self.count > 5:
                    # GPIO.output(self.electromagnet, False)
                    if dist < 1:
                        print("----DELIVERED-----", dist)
                    # print(self.distance_to_current_waypoint())
                    self.delivered = True
        if self.vehicle.commands.next>=4:
            self.state = "PrecisionLanding"


    def precision_landing_state_logic(self):
        if self.current_alt < 0.1:
            self.state = "Landed"
    
    def landed_state_logic(self):
        # self.vehicle.mode = VehicleMode('STABILIZE')
        # while self.vehicle.armed != False:
        #     self.vehicle.armed=False
        #     print("ent")
        # print("End of mission")
        self.vehicle.close()
        return True
    

    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint.
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint=self.vehicle.commands.next
        if nextwaypoint ==0:
            return None
        missionitem=self.cmds[nextwaypoint-1] #commands are zero indexed
        lat=missionitem.x
        lon=missionitem.y
        alt=missionitem.z
        targetWaypointLocation=LocationGlobalRelative(lat,lon,alt)
        
        distancetopoint = self.get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint

    def get_distance_metres(self,aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
                
if __name__ == '__main__':

    
    vehicle = Vehicle()
    vehicle.run()

