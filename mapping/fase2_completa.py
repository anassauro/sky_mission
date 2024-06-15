import dronekit
from dronekit import connect, VehicleMode
import cv2
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import time
import rospy
from fractions import Fraction
from PIL import Image as img
import piexif
from datetime import datetime
import argparse
import numpy as np
from cv2 import aruco
from pymavlink import mavutil


class Vehicle():

    def __init__(self):

        self.mission = Mission()
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
                    self.initial_state_logic()
                elif self.state == "PrecisionLanding":
                    self.precision_landing_state_logic()
                elif self.state == "Landed":
                    self.landed_state_logic()
            # Add more states and transitions as needed
    
    def initial_state_logic(self):
        # Logic for the Initial state
        print(self.vehicle.commands.next)
        if(self.vehicle.commands.next < 6 and self.vehicle.commands.next > 1):
                self.mission.save_pictures(self.vehicle.location.global_frame)
        if self.vehicle.commands.next>=8:
            self.state = "PrecisionLanding"


    def precision_landing_state_logic(self):
        # # Logic for the Precision Landing state
        # if self.vehicle.mode != 'LAND':
        #     self.vehicle.mode = VehicleMode('LAND')
        #     while self.vehicle.mode != 'LAND':
        #         time.sleep(1)
        #     print('vehicle in LAND mode')

        precision_landing = PrecLand(self, self.vehicle, 'aruco', marker_size, camera)
        rospy.spin()
        # while self.vehicle.location.global_frame.alt >= 0.2:
        #     ret, frame = self.mission.video().read()

        #     if not ret:
        #         continue
        #     precision_landing.msg_receiver(frame)
        self.state = "Landed"
    
    def landed_state_logic(self):
        self.vehicle.mode = VehicleMode('STABILIZE')
        while self.vehicle.armed != False:
            self.vehicle.armed=False
            print("ent")
        print("End of mission")
        self.vehicle.close()
                
class Mission():

    def __init__(self):
        self.quantidade_fotos = 0
        self.cam_frame = None
        self.capture = cv2.VideoCapture(2)
    
    def video(self):

        return self.capture

    def save_pictures(self, message):
        self.cam_frame = self.capture.read()[1]
        # self.cam_frame = cv2.resize(self.cam_frame, (960, 540))

        name = "/home/helena/26AGOSTO2023/simulation_images/image%d.jpg" % self.quantidade_fotos
        # output = "/home//Documents/tagged/image%d.jpg" % self.quantidade_fotos
        name_clean = "image%d.jpg" % self.quantidade_fotos
        latitude = message.lat  
        longitude = message.lon   
        cv2.imwrite(name, self.cam_frame)
        self.add_gps_metadata(name, latitude, longitude)

        print("Image {} at lat: {}, long: {}".format(self.quantidade_fotos, latitude, longitude))
        self.quantidade_fotos += 1
        time.sleep(2)
    
    def float_to_rational(self, f):
        f = Fraction(f).limit_denominator()
        return f.numerator, f.denominator

    def add_gps_metadata(self, image_path, latitude, longitude):
        exif_dict = piexif.load(image_path)

        lat_deg = abs(latitude)
        lat_min, lat_sec = divmod(lat_deg * 3600, 60)
        lat_deg, lat_min = divmod(lat_min, 60)
        lat_ref = 'N' if latitude >= 0 else 'S'

        lon_deg = abs(longitude)
        lon_min, lon_sec = divmod(lon_deg * 3600, 60)
        lon_deg, lon_min = divmod(lon_min, 60)
        lon_ref = 'E' if longitude >= 0 else 'W'

        lat_deg_num, lat_deg_den = self.float_to_rational(lat_deg)
        lat_min_num, lat_min_den = self.float_to_rational(lat_min)
        lat_sec_num, lat_sec_den = self.float_to_rational(lat_sec)

        lon_deg_num, lon_deg_den = self.float_to_rational(lon_deg)
        lon_min_num, lon_min_den = self.float_to_rational(lon_min)
        lon_sec_num, lon_sec_den = self.float_to_rational(lon_sec)

        gps_ifd = {
            piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
            piexif.GPSIFD.GPSLatitudeRef: lat_ref,
            piexif.GPSIFD.GPSLatitude: ((lat_deg_num, lat_deg_den), (lat_min_num, lat_min_den), (lat_sec_num, lat_sec_den)),
            piexif.GPSIFD.GPSLongitudeRef: lon_ref,
            piexif.GPSIFD.GPSLongitude: ((lon_deg_num, lon_deg_den), (lon_min_num, lon_min_den), (lon_sec_num, lon_sec_den)),
        }
        
        exif_dict['GPS'] = gps_ifd

        timestamp = datetime.now()
        timestamp_str = timestamp.strftime("%Y:%m:%d %H:%M:%S")

        exif_dict['0th'][piexif.ImageIFD.DateTime] = timestamp_str
        exif_dict['Exif'][piexif.ExifIFD.DateTimeOriginal] = timestamp_str
        exif_dict['Exif'][piexif.ExifIFD.DateTimeDigitized] = timestamp_str

        exif_bytes = piexif.dump(exif_dict)
        piexif.insert(exif_bytes, image_path)

        print("GPS metadata added to the image.")


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

    def __init__(self, instance,vehicle, target_type, target_size, camera_info):

        # Drone
        self.vehicle = vehicle
        self.instance = instance
        self.landed=False
        
        # Marker detector object
        self.detector = MarkerDetector(target_type, target_size, camera_info)

        # ROS node
        rospy.init_node('drone_node', anonymous=False)

        # Bridge ros-opencv
        self.bridge_object = CvBridge()

        # Post detection image publisher
        self.newimg_pub = rospy.Publisher('camera/colour/image_new', Image, queue_size=10)
        self.cam = Image()

        if self.landed == False:
            try:
                print("Criando subscriber...")
                self.subscriber = rospy.Subscriber('/webcam/image_raw', Image, self.msg_receiver)
            except:
                print('Erro ao criar subscriber!')

        self.teste = []

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

    #-- Callback
    def msg_receiver(self, message):

         # Bridge de ROS para CV
        cam = self.bridge_object.imgmsg_to_cv2(message,"bgr8")
        frame = cam
        # print(self.vehicle.location.global_relative_frame.alt)
        if self.vehicle.location.global_relative_frame.alt <=0.2:
            # print("Landed")
            self.landed = True
            self.instance.state = "Landed"
        
        if self.landed:
            self.vehicle.mode = VehicleMode('GUIDED')
            self.vehicle.armed=False
            time.sleep(15)
            print("end")
            # self.vehicle.mode = VehicleMode('LAND')

        # Look for the closest target in the frame
        closest_target = self.detector.aruco_detection(frame)

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
            

if __name__ == '__main__':

    
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
    vehicle = Vehicle()
    vehicle.run()

