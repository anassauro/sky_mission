from __future__ import print_function
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
from sensor_msgs.msg import Image
from pymavlink import mavutil # Needed for command message definitions
import time
import pandas
import ultralytics
import math


class Vehicle:

    def __init__(self, cap):
        
        self.cap = cap
        self.state = "Initial"
        self.current_wp = 0
        self.current_lat = 0
        self.current_lon = 0
        self.current_alt = 0
        self.vehicle_mode = ''
        self.started = False
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default = 'ttyUSB0:14550')
        args = parser.parse_args()

        #-- Connect to the vehicle
        print('Connecting...')
        self.vehicle = connect(args.connect)
        self.mission = PeopleDetection(self.vehicle)


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
            print("Reached waypoint %d" % message.seq)
            self.current_wp = message.seq
        
    def run(self):
        # Check current state and execute corresponding logic
            while True:
                print("Run")
                if self.state == "Initial":
                    print("Go to Initial")
                    self.initial_state_logic()
                elif self.state == "PrecisionLanding":
                    self.precision_landing_state_logic(cap)
                elif self.state == "Landed":
                    self.landed_state_logic()
            # Add more states and transitions as needed
    
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
    
    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint.
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint=self.vehicle.commands.next

        if nextwaypoint ==0:
            return None
        missionitem= self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat=missionitem.x
        lon=missionitem.y
        alt=missionitem.z
        targetWaypointLocation=LocationGlobalRelative(lat,lon,alt)
        distancetopoint = self.get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint
    
    def detection_state_logic(self):
        # Logic for the Initial state
        print(self.vehicle.commands.next)
        if(self.vehicle.commands.next < 4) and self.vehicle.commands.next > 2:
                self.mission.detection(self.cap)
        if self.vehicle.commands.next>=5:
            print("Initial")
            self.state = "PrecisionLanding"


    def precision_landing_state_logic(self,cap):
        # Logic for the Precision Landing state
        if self.vehicle.mode != 'LAND':
            self.vehicle.mode = VehicleMode('LAND')
            while self.vehicle.mode != 'LAND':
                time.sleep(1)
            print('vehicle in LAND mode')

        precision_landing = PrecLand(self.vehicle, 'aruco', marker_size, camera, cap)
        precision_landing.msg_receiver()
        
        # while self.vehicle.location.global_frame.alt >= 0.2:
        #     ret, frame = self.mission.video().read()

        #     if not ret:
        #         continue
        #     precision_landing.msg_receiver(frame)

        if self.current_alt<=0.2:
            self.state = "Landed"
    
    def landed_state_logic(self):
        print("End of mission")
        self.vehicle.close()

class PeopleDetection:

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.predictions = []
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', '/home/software/Downloads/fase4.pt')  # custom trained model
        self.model.conf = 0.65

    def detection(self,cap):

        ret, img = cap.read()  # or file, Path, PIL, OpenCV, numpy, list
        img = '/home/software/Documents/sd_card/image2_253.png'
        results = self.model(img)
        self.send_text(str(results))
        results.save(save_dir='results')
        print(self.predictions)
        print(results)

    def run(self):
            # Check current state and execute corresponding logic
            while True:
                print("Run")
                if self.state == "Initial":
                    print("Go to Initial")
                    self.initial_state_logic()
                elif self.state == "PrecisionLanding":
                    self.precision_landing_state_logic()
                elif self.state == "Landed":
                    self.landed_state_logic()
            # Add more states and transitions as needed
    
    def send_text(self, text):
        
            message = self.vehicle.message_factory.statustext_encode(mavutil.mavlink.MAV_SEVERITY_INFO, text.encode("utf-8"))            
            self.vehicle.send_mavlink(message)
            
    def goto_position_target_global_int(self, aLocation):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

        For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
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
        self.vehicle.send_mavlink(msg)
    
class MarkerDetector:

    def __init__(self, target_type, target_size, camera_info, cap):

        self.target_type = 'aruco'
        #self.target_type = target_type
        self.marker_size = target_size

        if self.target_type == 'aruco':
            self.dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
            self.parameters =  aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        elif self.target_type == 'qrcode':
            print("QR Code not implemented yet!")

        self.camera_matrix = camera_info[0]
        self.dist_coeff = camera_info[1]
        
        self.np_camera_matrix = np.array(self.camera_matrix)
        self.np_dist_coeff = np.array(self.dist_coeff)

        self.horizontal_res = camera_info[2][0]
        self.vertical_res = camera_info[2][1]

        self.horizontal_fov = camera_info[3][0]
        self.vertical_fov = camera_info[3][1]
    
    def pose_estimation(self, corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    
        nada, rvec, tvec = cv2.solvePnP(marker_points, corners, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        return rvec, tvec
    
    def aruco_detection(self):

        ret, frame = cap.read()
        # Marker detection
        markerCorners, markerIds, rejected = self.detector.detectMarkers(frame)

        i = 0
        if len(markerCorners) > 0: # if detect any Arucos

            closest_target = []
            closest_dist = 100000 # 1000 m (arbitrary large value)

            for corners in markerCorners: # For each Aruco

                marker_points = corners[0] # Vector with 4 points (x, y) for the corners

                # Draw points in image
                # final_image = self.draw_marker(frame, marker_points)
                final_image = frame

                # Pose estimation
                pose = self.pose_estimation(marker_points, self.marker_size, self.np_camera_matrix, self.np_dist_coeff)

                rvec, tvec = pose

                # 3D pose estimation vector
                x = round(tvec[0][0], 2)
                y = round(tvec[1][0], 2)
                z = round(tvec[2][0], 2)

                x_sum = marker_points[0][0] + marker_points[1][0] + marker_points[2][0] + marker_points[3][0]
                y_sum = marker_points[0][1] + marker_points[1][1] + marker_points[2][1] + marker_points[3][1]

                x_avg = x_sum / 4
                y_avg = y_sum / 4

                x_ang = (x_avg - self.horizontal_res*0.5)*self.horizontal_fov/self.horizontal_res
                y_ang = (y_avg - self.vertical_res*0.5)*self.vertical_fov/self.vertical_res

                payload = markerIds[i][0]
                i += 1
                
                # Check for the closest target
                if z < closest_dist:
                    closest_dist = z
                    closest_target = [x, y, z, x_ang, y_ang, payload, final_image]
            
            return closest_target
        return None
    
    def draw_marker(self, frame, points):
        topLeft, topRight, bottomRight, bottomLeft = points

        # Marker corners
        tR = (int(topRight[0]), int(topRight[1]))
        bR = (int(bottomRight[0]), int(bottomRight[1]))
        bL = (int(bottomLeft[0]), int(bottomLeft[1]))
        tL = (int(topLeft[0]), int(topLeft[1]))

        # Find the Marker center
        cX = int((tR[0] + bL[0]) / 2.0)
        cY = int((tR[1] + bL[1]) / 2.0)

        # Draw rectangle and circle
        rect = cv2.rectangle(frame, tL, bR, (0, 0, 255), 2)
        final = cv2.circle(rect, (cX, cY), radius=4, color=(0, 0, 255), thickness=-1)

        return final


class PrecLand:

    def __init__(self, vehicle, target_type, target_size, camera_info,cap):

        self.cap = cap
        # Drone
        self.vehicle = vehicle
        
        # Marker detector object
        self.detector = MarkerDetector(target_type, target_size, camera_info, cap)

        self.teste = []
    
    def msg_receiver(self):

         # Bridge de ROS para CV
        #cam = self.bridge_object.imgmsg_to_cv2(message,"bgr8")
        ret, frame = self.cap.read()

        # Look for the closest target in the frame
        closest_target = self.detector.aruco_detection()

        if closest_target is not None:

            if self.vehicle.mode != 'LAND':
                self.vehicle.mode = VehicleMode('LAND')
                while self.vehicle.mode != 'LAND':
                    print(self.vehicle.mode)
                    time.sleep(1)
                print('vehicle in LAND mode')
            
            x, y, z, x_ang, y_ang, payload, draw_img = closest_target

            # times = time.time()*1e6
            dist = float(z)/100

            # AQUI
            self.send_land_message(x_ang, y_ang, dist)

            print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')

    def send_land_message(self, x_ang,y_ang,dist_m,time=0):
        msg = self.vehicle.message_factory.landing_target_encode(
            time,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            x_ang,
            y_ang,
            dist_m,
            0,
            0,
        )
        self.vehicle.send_mavlink(msg)
        print("Mensagem enviada")

if __name__ == '__main__':

    cap = cv2.VideoCapture(0)
    marker_size = 25

    # Camera infos
    
    # camera_matrix = [[536.60468864,   0.0,         336.71838244],
    #                [  0.0,        478.13866264, 353.24213721],
    #                [  0.0,         0.0,        1.0        ]]


    # dist_coeff = [0.0, 0.0, 0.0, 0.0, 0] # Camera distortion matrix
    # res = (640,480) # Camera resolution in pixels
    # fov = (1.15976, 0.907) # Camera FOV

    # camera = [camera_matrix, dist_coeff, res, fov]

        # Target size in cm
    marker_size = 50

    # Camera infos
    camera_matrix = [[467.74270306499267, 0.0, 320.5],
                    [0.0, 467.74270306499267, 240.5],
                    [0.0, 0.0, 1.0]]

    dist_coeff = [0.0, 0.0, 0.0, 0.0, 0] # Camera distortion matrix
    res = (640, 480) # Camera resolution in pixels
    fov = (1.2, 1.1) # Camera FOV
        
    camera = [camera_matrix, dist_coeff, res, fov]
    vehicle = Vehicle(cap)
    vehicle.run()   
