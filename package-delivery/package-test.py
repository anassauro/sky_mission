import dronekit
from dronekit import connect, VehicleMode
import cv2
import argparse
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import numpy as np
from cv2 import aruco
from pymavlink import mavutil
import RPi.GPIO as GPIO

class Vehicle():

    def __init__(self):

        self.state = "Initial"
        self.current_wp = 0
        self.current_lat = 0
        self.current_lon = 0
        self.current_alt = 0
        self.vehicle_mode = ''
        self.started = False
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default = 'tcp:127.0.0.1:5763')
        args = parser.parse_args()

        self.electromagnet = 11
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.electromagnet, GPIO.OUT)
        GPIO.output(self.electromagnet, True)
        self.delivered = False

        #-- Connect to the vehicle
        print('Connecting...')
        self.vehicle = connect(args.connect)

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
                print("Run")
                if self.state == "Initial":
                    print("Go to Initial")
                    self.delivery_state_logic()
                elif self.state == "PrecisionLanding":
                    self.precision_landing_state_logic()
                elif self.state == "Landed":
                    self.landed_state_logic()
            # Add more states and transitions as needed
    
    def delivery_state_logic(self):
        # Logic for the Initial state
        print(self.vehicle.commands.next)
        if(self.vehicle.commands.next == 3) and self.delivered == False:
                GPIO.output(self.electromagnet, False)
                print("----DELIVERED-----")
                self.delivered = True
        if self.vehicle.commands.next>=4:
            self.state = "PrecisionLanding"


    def precision_landing_state_logic(self):

        self.state = "Landed"
    
    def landed_state_logic(self):
        self.vehicle.mode = VehicleMode('STABILIZE')
        while self.vehicle.armed != False:
            self.vehicle.armed=False
            print("ent")
        print("End of mission")
        self.vehicle.close()
                
if __name__ == '__main__':

    
    vehicle = Vehicle()
    vehicle.run()

