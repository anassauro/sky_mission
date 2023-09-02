from __future__ import print_function
import rospy
import cv2
import time
import numpy as np
from cv2 import aruco
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from cv_bridge import CvBridge 
import argparse
import torch
import matplotlib.pyplot as plt
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '127.0.0.1:14550')
args = parser.parse_args()

#-- Connect to the vehicle
print('Connecting...')
vehicle = connect(args.connect)

class peopleDetector:

    def __init__(self, vehicle):
        
        self.vehicle = vehicle
        self.cap = cv2.VideoCapture(0)

    
    def detection(self,frame):

        model = torch.hub.load('ultralytics/yolov5', 'custom', path='path_to_weights.pt')
        img = frame  # or file, Path, PIL, OpenCV, numpy, list
        results = model(img)
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.imshow(results.render()[0])
        plt.show()

    
    def goto_position_target_global_int(self, aLocation):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

        For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0, # X velocity in NED frame in m/s
            0, # Y velocity in NED frame in m/s
            0, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        vehicle.send_mavlink(msg)
    
    def get_location_metres(self, original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
        else:
            raise Exception("Invalid Location object passed")
            
        return targetlocation

    def get_distance_metres(aLocation1, aLocation2):

        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
    def goto(self,dNorth, dEast,gotoFunction=vehicle.simple_goto):
        """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
        the target position. This allows it to be called with different position-setting commands. 
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """
        
        currentLocation = vehicle.location.global_relative_frame
        targetLocation = self.get_location_metres(currentLocation, dNorth, dEast)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        gotoFunction(targetLocation)
        
        #print "DEBUG: targetLocation: %s" % targetLocation
        #print "DEBUG: targetLocation: %s" % targetDistance

        while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
            #print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance=self.get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
                print("Reached target")
                break
            time.sleep(2)
    
    
    


    
if __name__ == '__main__':

    #-- Check vehicle status
    print(f"Mode: {vehicle.mode.name}")
    print(" Global Location: %s" % vehicle.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print(" Local Location: %s" % vehicle.location.local_frame)
    print(" Attitude: %s" % vehicle.attitude)
    print(" Velocity: %s" % vehicle.velocity)
    print(" Gimbal status: %s" % vehicle.gimbal)
    print(" EKF OK?: %s" % vehicle.ekf_ok)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Rangefinder: %s" % vehicle.rangefinder)
    print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Armed: %s" % vehicle.armed)    # settable

    while not vehicle.armed:
        print (" Waiting for arming...")
        print ("Taking off!")

    aTargetAltitude = 5
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    time.sleep(1)

    detection = peopleDetector(vehicle)