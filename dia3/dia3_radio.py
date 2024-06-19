import dronekit
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
import math
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



global UPPER_BLUE
global LOWER_BLUE 

global UPPER_ORANGE 
global LOWER_ORANGE 

global ERODE_KERNEL
global DILATE_KERNEL

global MIN_AREA

MIN_AREA = 75*75*1.5

ERODE_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
DILATE_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (75, 75))

UPPER_BLUE = (140,255,255) 
LOWER_BLUE = (100,150,0)

UPPER_ORANGE = (225, 250, 255)
LOWER_ORANGE = (0, 100, 45)

global capture
global SIMULATION

RASP = False
if RASP:
    import RPi.GPIO as GPIO


SIMULATION = False
capture = cv2.VideoCapture(4)

class Vehicle():

    def __init__(self):
        global RASP

        if RASP:
            self.electromagnet = 11
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.electromagnet, GPIO.OUT)

        self.mission = Mission()
        self.state = "Initial"
        self.current_wp = 0
        self.current_lat = 0
        self.current_lon = 0
        self.current_alt = 0
        self.count = 0
        self.vehicle_mode = ''
        self.started = False
        self.returning = False
        self.capture = capture

        self.md = MarkerDetector('aruco', marker_size, camera)
        
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default = 'tcp:127.0.0.1:5763')
        args = parser.parse_args()

        #-- Connect to the vehicle
        print('Connecting...')
        self.vehicle = connect(args.connect)
        self.commands = self.vehicle.commands
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
                print("Look for aruco...")
                frame = self.capture.read()[1] 
                aruco = self.md.aruco_detection(frame)
                if aruco:
                    print("--- \n ARUCO FOUND! \n --- \n \n")

                if self.vehicle_mode == "GUIDED":
                    print("jogando caixa")

                    if RASP:
                        GPIO.output(self.electromagnet, GPIO.LOW)

                

                """
                if self.state == "Initial":
                    print("Go to Initial")
                    self.initial_state_logic()
                elif self.state == "ReturnToAruco":
                    print("Going to Aruco")
                    self.back_to_aruco()
                elif self.state == "PrecisionLanding":
                    self.precision_landing_state_logic()
                elif self.state == "Landed":
                    self.landed_state_logic()

                """
            # Add more states and transitions as needed
    
    def initial_state_logic(self):
        # Logic for the Initial state
        print(self.vehicle.commands.next)
        if(self.vehicle.commands.next < 100 and self.vehicle.commands.next > 1):
                
                frame = self.capture.read()[1]
                #frame = cv2.resize(frame, (960, 540))

                self.mission.save_pictures(self.vehicle.location.global_frame, frame)
                self.mission.image_processing(self.vehicle.location.global_frame, frame)
        if self.vehicle.commands.next>=100:
            self.state = "ReturnToAruco"


    def precision_landing_state_logic(self):
        # # Logic for the Precision Landing state
        # if self.vehicle.mode != 'LAND':
        #     self.vehicle.mode = VehicleMode('LAND')
        #     while self.vehicle.mode != 'LAND':
        #         time.sleep(1)
        #     print('vehicle in LAND mode')

        precision_landing = PrecLand(self, self.vehicle, 'aruco', marker_size, camera)
        if SIMULATION:
            rospy.spin()
        # while self.vehicle.location.global_frame.alt >= 0.2:
        #     ret, frame = self.mission.video().read()

        #     if not ret:
        #         continue
        #     precision_landing.msg_receiver(frame)
        self.state = "Landed"
    
    def back_to_aruco(self):
        time.sleep(1)
        lat = -35.3631719
        lon = 149.1652103
        altitude = 10
        self.count = self.count + 1
        if self.returning == False and self.vehicle_mode != 'GUIDED':
            # self.commands.clear()
            self.vehicle.mode = VehicleMode("GUIDED")
            lat = -35.3631719
            lon = 149.1652103
            altitude = 10
            # cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            # 0, 0, 0, 0, 0, 0,
            # lat, lon, altitude)
            point = LocationGlobalRelative(lat,lon,altitude)
            self.vehicle.simple_goto(point)
            # self.commands.add(cmd)
            # self.commands.upload()
            # self.vehicle.flush()
            self.returning = True

        dist = self.distance_to_current_waypoint(lat,lon,altitude)
        if dist < 1 and self.count > 5:
            print("----REACHED WAYPOINT----", dist)
            self.state = "PrecisionLanding"
        
        #LOGIC HERE

    def landed_state_logic(self):
        self.vehicle.mode = VehicleMode('STABILIZE')
        while self.vehicle.armed != False:
            self.vehicle.armed=False
            print("ent")
        print("End of mission")
        self.vehicle.close()

    def distance_to_current_waypoint(self,lat,lon,altitude):
        """
        Gets distance in metres to the current waypoint.
        It returns None for the first waypoint (Home location).
        """
        # nextwaypoint=self.vehicle.commands.next
        # if nextwaypoint ==0:
        #     return None
        # missionitem=self.commands[nextwaypoint-1] #commands are zero indexed
        # lat=missionitem.x
        # lon=missionitem.y
        # alt=missionitem.z

        targetWaypointLocation=LocationGlobalRelative(lat,lon,altitude)
        
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
                
class Mission():

    def __init__(self):
        global capture
        self.quantidade_fotos = 0
        self.cam_frame = None
        self.capture = capture

        self.md = MarkerDetector('aruco', marker_size, camera)
        self.aruco_dict = dict()
    
    def video(self):

        return self.capture

    def save_pictures(self, message, frame):
        
        name = "./images/image%d.jpg" % self.quantidade_fotos
        # output = "/home//Documents/tagged/image%d.jpg" % self.quantidade_fotos
        # name_clean = "image%d.jpg" % self.quantidade_fotos
        latitude = message.lat  
        longitude = message.lon   
        cv2.imwrite(name, frame)
        self.add_gps_metadata(name, latitude, longitude)

        print("Image {} at lat: {}, long: {}".format(self.quantidade_fotos, latitude, longitude))
        self.quantidade_fotos += 1
        time.sleep(2)
    
    def float_to_rational(self, f):
        f = Fraction(f).limit_denominator()
        return f.numerator, f.denominator
    
    def is_square(self, rect, tol = 0.5 ) -> bool:    
        print(rect)
        aux = rect[2]/rect[3]
        print(aux)
        if(aux >= (1 - tol) and aux <= (1 + tol)):
            print("is square")
            return True
        return False
    
    def image_processing(self, message, frame, RESIZE_FACTOR = 1):
        #LOGIC HERE

        img = frame
        height, width, _ = frame.shape

        #FILTERING BY COLOR
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(img, LOWER_BLUE, UPPER_BLUE)
        mask_orange = cv2.inRange(img, LOWER_ORANGE, UPPER_ORANGE)

        masks = cv2.bitwise_or(mask_blue, mask_orange)

        #APPLYING MASKS, REMOVE IF USING HOUGH TRANSFORM
        masks = cv2.erode(masks, ERODE_KERNEL)
        masks = cv2.dilate(masks, DILATE_KERNEL)

        cnts, _ = cv2.findContours(masks, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #FILTER BY MIN AREA
        filtered_boxes = [cv2.boundingRect(cnt) for cnt in cnts if cv2.contourArea(cnt) > MIN_AREA and self.is_square(cv2.boundingRect(cnt))]
        
        #SEARCH FOR ARUCO
        for box in filtered_boxes:
            print("searching for aruco in picture...")
            aruco = self.md.aruco_detection(frame[box[1]: box[1] + box[3], box[0]: box[0] + box[2]])
            cv2.imshow("ARUCO", frame[box[1]: box[1] + box[3], box[0]: box[0] + box[2]])
            cv2.waitKey(1)
                
            if aruco:
                distance = int(math.dist((height/2, width/2), (box[0] + box[2]/2, box[1] + box[3]/2)))
                print("FOUND ARUCO FUCK YEAH ID", aruco[5])
                if aruco[5] in self.aruco_dict:
                    print("ARUCO ID ALREADY IN. CHECKING IF IT HAS BETTER CORDS")
                    if self.aruco_dict.get(aruco[5])[2] > distance:
                        print("BETTER CENTRALIZED, SWAPING CORDS")
                        latitude = message.lat  
                        longitude = message.lon
                        self.aruco_dict.update({aruco[5] : [latitude, longitude, distance]})
                else:
                    latitude = message.lat  
                    longitude = message.lon
                    self.aruco_dict.update({aruco[5] : [latitude, longitude, distance]})


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
            self.parameters =  aruco.DetectorParameters_create()
            #self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

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

        markerCorners, markerIds, rejected = aruco.detectMarkers(
            frame, self.dictionary, parameters=self.parameters
        )

        if markerIds is not None and len(markerCorners) > 0:
            closest_target = []
            closest_dist = float('inf')

            for i, corners in enumerate(markerCorners):
                marker_points = corners[0]
                final_image = frame

                try:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        [marker_points], self.marker_size, self.np_camera_matrix, self.np_dist_coeff
                    )

                    x = round(tvec[0][0][0], 2)
                    y = round(tvec[0][0][1], 2)
                    z = round(tvec[0][0][2], 2)

                    x_avg = np.mean(marker_points[:, 0])
                    y_avg = np.mean(marker_points[:, 1])

                    x_ang = (x_avg - self.horizontal_res * 0.5) * self.horizontal_fov / self.horizontal_res
                    y_ang = (y_avg - self.vertical_res * 0.5) * self.vertical_fov / self.vertical_res

                    payload = markerIds[i][0]

                    if z < closest_dist:
                        closest_dist = z
                        closest_target = [x, y, z, x_ang, y_ang, payload, final_image]
                except Exception as e:
                    print(f"Error in pose estimation: {e}")
            
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

        global capture
        # Drone
        self.vehicle = vehicle
        self.instance = instance
        self.landed=False
        
        # Marker detector object
        self.detector = MarkerDetector(target_type, target_size, camera_info)

        if not SIMULATION:
            self.cam = capture
            self.main_loop(capture)
        else:
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
    

    def main_loop(self,cap):

        while self.instance.state != "Landed":

            frame = cap.read()[1]
            # print(self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt <=0.2:
                # print("Landed")
                self.landed = True
                self.instance.state = "Landed"
            
            if self.landed:
                self.vehicle.mode = VehicleMode('GUIDED')
                self.vehicle.armed=False
                time.sleep(15)
                print("End")
                # self.vehicle.mode = VehicleMode('LAND')

            # Look for the closest target in the frame
            closest_target = self.detector.aruco_detection(frame)

            if closest_target is not None:

                if self.vehicle.mode != 'LAND':
                    self.vehicle.mode = VehicleMode('LAND')
                    while self.vehicle.mode != 'LAND':
                        print(self.vehicle.mode)
                        time.sleep(0.1)
                    print('vehicle in LAND mode')

                
                x, y, z, x_ang, y_ang, payload, draw_img = closest_target

                # times = time.time()*1e6
                dist = float(z)/100

                # AQUI
                self.send_land_message(x_ang, y_ang, dist)

                print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')
            
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
            print("End")
            # self.vehicle.mode = VehicleMode('LAND')

        # Look for the closest target in the frame
        closest_target = self.detector.aruco_detection(frame)

        if closest_target is not None:

            if self.vehicle.mode != 'LAND':
                self.vehicle.mode = VehicleMode('LAND')
                while self.vehicle.mode != 'LAND':
                    print(self.vehicle.mode)
                    time.sleep(0.1)
                print('vehicle in LAND mode')

            
            x, y, z, x_ang, y_ang, payload, draw_img = closest_target

            # times = time.time()*1e6
            dist = float(z)/100

            # AQUI
            self.send_land_message(x_ang, y_ang, dist)

            print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')

if __name__ == '__main__':

    # Camera infos
    if SIMULATION:
        marker_size = 25

        camera_matrix = [[536.60468864,   0.0,         336.71838244],
                   [  0.0,        478.13866264, 353.24213721],
                   [  0.0,         0.0,        1.0        ]]

        dist_coeff = [0.0, 0.0, 0.0, 0.0, 0] # Camera distortion matrix
        res = (640,480) # Camera resolution in pixels
        fov = (1.15976, 0.907) # Camera FOV

        camera = [camera_matrix, dist_coeff, res, fov]
    else:
        camera_matrix= [[629.60088304,  0.0,         649.96450783],
    [  0.0,         628.99975883, 323.37037351],
    [  0.0,           0.0,           1.0        ]]
        dist_coeff = [[-0.08654266,  0.00064634, -0.01367921,  0.00537603,  0.00417901]]

        # Target size in cm
        marker_size = 35

        res = (1280, 720) # Camera resolution in pixels
        fov = (1,58717, 1.03966) # Camera FOV
        
        camera = [camera_matrix, dist_coeff, res, fov]
    vehicle = Vehicle()
    vehicle.run()

