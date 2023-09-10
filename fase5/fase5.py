import cv2
import time
import numpy as np
from cv2 import aruco
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from cv_bridge import CvBridge 
import argparse
from simple_pid import PID

WP1 = LocationGlobalRelative(50.9102414, 6.2255084,15)
WP2 = LocationGlobalRelative(50.9114878, 6.2276513,15)




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
        parser.add_argument('--connect', default = 'tcp:127.0.0.1:5763')
        args = parser.parse_args()

        #-- Connect to the vehicle
        print('Connecting...')
        self.vehicle = connect(args.connect)
        self.mission = followAruco(self.vehicle)


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
            self.mission.mission_duration = 0
            self.initial_time = time.time()
        # Check current state and execute corresponding logic
            while True:
                print("Run")
                if self.state == "Initial": 
                    print("Go to Initial")
                    self.initial_state_logic()
                    self.mission.mission_duration = time.time() - self.initial_time
                    print(self.mission.mission_duration)
                elif self.state == "PrecisionLanding":
                    self.precision_landing_state_logic()
                elif self.state == "Landed":
                    self.landed_state_logic()
            # Add more states and transitions as needed
    
    def initial_state_logic(self):
        # Logic for the Initial state
        
        
        print(self.vehicle.commands.next)
        if self.mission.mission_duration <= 15:
            self.mission.detection(cap)
            print("Tracking aruco")
        else:
            print("Initial")
            self.state = "PrecisionLanding"


    def precision_landing_state_logic(self):
        # Logic for the Precision Landing state
        if self.vehicle.mode != 'LAND':
            self.vehicle.mode = VehicleMode('LAND')
            while self.vehicle.mode != 'LAND':
                time.sleep(1)
            print('vehicle in LAND mode')

        precision_landing = PrecLand(self.vehicle, 'aruco', marker_size, camera, cap)
        precision_landing.msg_receiver()


        if self.current_alt<=0.2:
            self.state = "Landed"
    
    def landed_state_logic(self):
        print("End of mission")
        self.cap.release()
        self.vehicle.close()

class followAruco:

    def __init__(self, vehicle):

        # Drone
        self.vehicle = vehicle
        
        # Marker detector object
        self.mission_duration = 0
        
        self.cap = cv2.VideoCapture(0)


    def move_drone_with_velocity(self,vx, vy, vz):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.
        """
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b110111000111, # type_mask (only positions enabled)
            0, 0, 0,
            vx, vy,vz, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        vehicle.send_mavlink(msg)
    
    
    def visual_servoing_control(self,corners, frame):     

        # Initialize PID controllers for lateral and forward control
        pid_x = PID(Kp=0.005, Ki=0.005, Kd=0.25, setpoint=0)
        pid_y = PID(Kp=0.005, Ki=0.005, Kd=0.25, setpoint=0)

        if corners:
            # Assuming the first detected marker's center is our target
            marker_center = corners[0][0].mean(axis=0)

            # Calculate the error between the marker center and the image center
            image_center = np.array([frame.shape[1] / 2, frame.shape[0] / 2])
            error = marker_center - image_center
            print(error)
            # Calculate the desired velocity commands (lateral and forward) using PID controllers
            vx = +pid_x(error[1])  # Drone moves in the opposite direction to align with the marker's center (left/right)
            vy = -pid_y(error[0])

            # Visual servoing parameters (adjust as needed)
            vz = 0  # Desired vertical velocity (m/s)
            duration = 0.5 # Duration of each movement command (in seconds)

            # Move the drone with velocity commands
            self.move_drone_with_velocity(vx, vy, vz)


    #-- Callback
    def detection(self, cap):

        
        #frame = self.bridge_object.imgmsg_to_cv2(message,"bgr8")
        success,frame = cap.read() 
        #frame = cv2.flip(frame,1)
        #cv2.imshow("frame", frame)
        # Look for the closest target in the frame

        self.dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.parameters =  aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        corners, markerIds, rejected = self.detector.detectMarkers(frame)


                
        if corners:
            print("cheguei")
            self.visual_servoing_control(corners,frame)


        self.cap.release()
        cv2.destroyAllWindows()





class MarkerDetector:

    def __init__(self, target_type, target_size, camera_info):

        self.target_type = target_type
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
    
    def aruco_detection(self, frame):

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
            
            return closest_target, markerCorners, final_image
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
        self.detector = MarkerDetector(target_type, target_size, camera_info)

        self.teste = []
    
    def msg_receiver(self):

         # Bridge de ROS para CV
        #cam = self.bridge_object.imgmsg_to_cv2(message,"bgr8")
        ret, frame = self.cap.read()

        # Look for the closest target in the frame
        aruco = self.detector.aruco_detection(frame)

        if aruco is not None:

            closest_target, corners, final_image = self.detector.aruco_detection(frame)


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