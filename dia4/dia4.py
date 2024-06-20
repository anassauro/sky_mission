import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import math
import cv2
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
import time
import rospy
from fractions import Fraction
from PIL import Image as img
from datetime import datetime
import argparse
import numpy as np
from cv2 import aruco
from pymavlink import mavutil


global capture
global SIMULATION
global electromagnet
global state
global landed
global found_aruco
global ID_TAKEOFF
global ID_OBJECT
global FIRST_TASK
global DELIVERED

found_aruco = False
landed = False
# Mude para False caso seja teste na Rasp
SIMULATION = True

state = "Searching"

if not SIMULATION:
    import RPi.GPIO as GPIO
    electromagnet = 11
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(electromagnet, GPIO.OUT)
    GPIO.output(electromagnet, True)

# Mude aqui o ID do Objeto Certo -> Use o aruco_detect.py (no próprio pc ou no drone desarmado mesmo) para descobrir o ID do ArUco certo para pousar

ID_OBJECT = 4
ID_TAKEOFF = 3

# Mude aqui dependendo da parte da tarefa sendo feita

FIRST_TASK = True
DELIVERED = False

# Se FIRST_TASK for True, ele tentará pousar no ArUco ID_OBJECT. Caso contrário, ele vai procurar o ArUco de Takeoff e soltar um pacote nele.

"""
Recomendações:

Para a primeira task, faça um survey com altura baixa para tentar garantir apenas um ArUco por frame. Inicialize o código e ele já vai começar a procurar o ArUco.
Inicialize a missão na QGround. O drone irá pousar quando encontrar o ArUco correto.

Para a segunda task, pare o código. Coloque na QGround um waypoint para o takeoff. Missão simples: 
1. Drone dá takeoff a uma altura pequena (3/4/5 m)
2. Waypoint para a coordenada da base de takeoff (peguem com GPS do Maps ou outro e verifiquem se realmente faz sentido... Chequem a coordenada em relação a Fence!).
3. Rode o código. Ele vai começar a procurar já o ArUco com ID_TAKEOFF para fazer um Centralize (PLND com limite de altura) e soltar o pacote.
4. Pouse o drone como quiser... RC pode assumir depois da entrega do pacote.


"""

if not SIMULATION:
    capture = cv2.VideoCapture(0)

class Vehicle():

    def __init__(self,target_type, target_size, camera_info):

        self.target_type = target_type
        self.target_size = target_size
        self.camera_info = camera_info
        self.current_wp = 0
        self.current_lat = 0
        self.current_lon = 0
        self.current_alt = 0
        self.count = 0
        self.started = False
        self.returning = False
        
        parser = argparse.ArgumentParser()

        if SIMULATION:
            parser.add_argument('--connect', default = 'tcp:127.0.0.1:5763')
        else:
            parser.add_argument('--connect', default = '/dev/ttyACM0')

        args = parser.parse_args()

        #-- Connect to the vehicle
        print('Connecting...')
        self.vehicle = connect(args.connect)
        self.mission = Mission(self,self.vehicle,self.target_type, self.target_size, self.camera_info)
        
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
            self.vehicle.mode = 'GUIDED'
            # Check current state and execute corresponding logic
            while True:
                if state == "Searching":
                    self.searching_logic()
                elif state == "PrecisionLanding":
                    self.precision_landing_state_logic()
                elif state == "PrecisionDelivery":
                    self.precision_landing_state_logic()
                elif state == "Landed":
                    self.landed_state_logic()
            # Add more states and transitions as needed
    
    def searching_logic(self):

        if not SIMULATION:
           
           while not found_aruco:
               cap = capture
               frame = cap.read()[1]
               self.mission.aruco_search(frame)
        else:
           rospy.spin()

        if FIRST_TASK and found_aruco:
            self.state = "PrecisionLanding"
        elif not FIRST_TASK and found_aruco:
            self.state = "PrecisionDelivery"

    
    def precision_package_delivery_logic(self):

        if SIMULATION:
            rospy.spin()
        else:
            while not landed:
                cap = capture
                frame = cap.read()[1]
                self.mission.precision_delivery(frame)

        self.state = "Landed"

    def precision_landing_state_logic(self):

        
        if SIMULATION:
            rospy.spin()
        else:

            while not landed:
                cap = capture
                frame = cap.read()[1]
                self.mission.precision_landing(frame)

        self.state = "Landed"


    def landed_state_logic(self):
        self.vehicle.mode = VehicleMode('STABILIZE')
        while self.vehicle.armed != False:
            self.vehicle.armed=False
            print("Disarmed")
        print("End of mission")
        self.vehicle.close()
    
                
class Mission():

    def __init__(self, instance, vehicle, target_type, target_size, camera_info):
        
        self.cam_frame = None
        self.vehicle = vehicle
        self.instance = instance
        self.landed=False
        self.last_goto_time = None
        self.last_dist = 1
        self.last_target = None
        self.marker_detector = MarkerDetector(target_type, target_size, camera_info)
        

        if SIMULATION:

            rospy.init_node('drone_node', anonymous=False)

            # Bridge ros-opencv
            self.bridge_object = CvBridge()

            # Post detection image publisher
            self.newimg_pub = rospy.Publisher('camera/colour/image_new', Image, queue_size=1)
            self.cam = Image()

            if self.landed == False:
                try:
                    print("Criando subscriber...")
                    self.subscriber = rospy.Subscriber('/webcam/image_raw', Image, self.msg_receiver, queue_size=1,buff_size=2**24)
                except:
                    print('Erro ao criar subscriber!')

        self.teste = []
    
    def goto(self,dNorth, dEast):

        currentLocation=self.vehicle.location.global_relative_frame
        targetLocation=self.get_location_metres(currentLocation, dNorth, dEast)
        targetDistance=self.get_distance_metres(currentLocation, targetLocation)
        self.vehicle.simple_goto(targetLocation)

        while self.vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
            remainingDistance=self.get_distance_metres(self.vehicle.location.global_frame, targetLocation)
            print ("Distance to target: ", remainingDistance)

            ## AQUI NÃO FOI MUITO TESTADO, ISSO SERVE PARA DAR BREAK CASO O DRONE FIQUE PARADO NO MESMO LUGAR...
            if abs(remainingDistance -self.last_dist) < 0.05:
                break

            self.last_dist = remainingDistance
            if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
                print ("Reached target")
                break
            time.sleep(2)

    def get_location_metres(self,original_location, dNorth, dEast):
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
    
    def get_distance_metres(self,aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
    
    def aruco_search(self,frame): 
        global state, found_aruco

        closest_target = self.marker_detector.aruco_detection(frame)

        if closest_target is not None:
            x, y, z, x_ang, y_ang, payload, draw_img = closest_target
            # print("Altitude",self.vehicle.location.global_relative_frame.alt)
            print(f'MARKER DETECTED: ID = {payload}')
            if payload == ID_OBJECT and FIRST_TASK == True:
                found_aruco = True
                print("Found right OBJECT marker")
                if SIMULATION:
                    state = "PrecisionLanding"
            elif payload == ID_TAKEOFF and FIRST_TASK == False:
                found_aruco = True
                print("Found right TAKEOFF marker")
                if SIMULATION:
                    state = "PrecisionDelivery"
            else:
                print("Wrong marker")

        return True
    
    def msg_receiver(self,message): 
        global state
            
        cam = self.bridge_object.imgmsg_to_cv2(message,"bgr8")
        frame = cam
        
        if state == "PrecisionLanding":

            self.precision_landing(frame)

        elif state == "Searching":

            self.aruco_search(frame)
        
        elif state == "PrecisionDelivery":

            self.precision_delivery(frame)
    
    
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

    def precision_landing(self,frame):

        global landed, state
        
        if self.vehicle.location.global_relative_frame.alt < 0.2:

            landed = True
            state = "Landed"

            # Look for the closest target in the frame
        closest_target = self.marker_detector.aruco_detection(frame)

        if closest_target is not None:
            x, y, z, x_ang, y_ang, payload, draw_img = closest_target
            print("Altitude",self.vehicle.location.global_relative_frame.alt)

            if self.vehicle.mode != 'LAND':
                self.vehicle.mode = VehicleMode('LAND')
                while self.vehicle.mode != 'LAND':
                    print(self.vehicle.mode)
                    # time.sleep(0.1)
                print('vehicle in LAND mode')


            # times = time.time()*1e6
            dist = float(z)/100
            
            # AQUI
            self.send_land_message(x_ang, y_ang, dist)

            print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')

    def precision_delivery(self,frame):

        global electromagnet, DELIVERED, state, landed

        if DELIVERED:
            self.vehicle.mode = VehicleMode('GUIDED')
            self.goto(0.3,0)
            time.sleep(5)

            while not landed:
                if self.vehicle.mode != 'LAND':

                    self.vehicle.mode = VehicleMode('LAND')

                if self.vehicle.location.global_relative_frame.alt < 0.2:

                    landed = True
                    state = "Landed"

            # Look for the closest target in the frame
        closest_target = self.marker_detector.aruco_detection(frame)


        if closest_target is not None:
                x, y, z, x_ang, y_ang, payload, draw_img = closest_target
                print("Altitude",self.vehicle.location.global_relative_frame.alt)

                # Change delivery altitude as you wish :)

                if self.vehicle.location.global_relative_frame.alt > 0.7:


                    if self.vehicle.mode != 'LAND':
                        self.vehicle.mode = VehicleMode('LAND')
                        while self.vehicle.mode != 'LAND':
                            print(self.vehicle.mode)
                        print('---Vehicle in LAND mode---')

                    dist = float(z)/100
                    
                    # Mandando duas vezes a mensagem para inicializar com certeza o PLND
                    self.send_land_message(x_ang, y_ang, dist)
                    self.send_land_message(x_ang,y_ang,dist)

                else:
                    print("---Delivering package---")
                    # Change to Loiter so RC can assume too
                    if self.vehicle.mode == 'LAND':
                        self.vehicle.mode = VehicleMode('GUIDED')
                        DELIVERED = True
                    if not SIMULATION:
                        GPIO.output(electromagnet, False)
                        DELIVERED = True


                print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')

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

        # PLND 

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
        fov = (1.58717, 1.03966) # Camera FOV
        
        camera = [camera_matrix, dist_coeff, res, fov]
    vehicle = Vehicle('aruco', marker_size, camera)
    vehicle.run()
