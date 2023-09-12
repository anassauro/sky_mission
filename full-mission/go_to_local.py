import rospy
import time
import math
import dronekit
from pymavlink import mavutil

#from actuator import Actuator
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Vector3
from std_msgs.msg import String, Header
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler, euler_from_quaternion


# Overall mission altitude
ALTITUDE = 1.5

# (x, y, z) pickup estimate location
PICKUP_POINT = [0, 0, ALTITUDE]

# (x, y, z) transit estimate location
TRANSIT_POINT = [0, -2, ALTITUDE]

# (x, y, z) pickup estimate location
DROP_POINT = [0, -10, 1]

# (x, y, z) go back 1
GO_BACK_1 = [-4, -10, ALTITUDE]

# (x, y, z) go back 2
GO_BACK_2 = [-4, 0, ALTITUDE]

# Number os arucos detected to avoid false positives
PICKUP_ROBUST_PRECLAND = 5

# Activate line only yaw
ONLY_YAW = False

# Activate only centralize
ONLY_CENTRALIZE = 0

# Number os lines detected to avoid false positives
TRANSIT_ROBUST_LINEFOLLOWER = 10

def quaternions_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return [X, Y, Z]

def euler_angle_to_quaternions(roll, pitch, yaw):
    cy = math.cos(math.radians(yaw) * 0.5)
    sy = math.sin(math.radians(yaw) * 0.5)
    cr = math.cos(math.radians(roll) * 0.5)
    sr = math.sin(math.radians(roll) * 0.5)
    cp = math.cos(math.radians(pitch) * 0.5)
    sp = math.sin(math.radians(pitch) * 0.5)

    w = cy * cr * cp + sy * sr * sp
    x = cy * sr * cp - sy * cr * sp
    y = cy * cr * sp + sy * sr * cp
    z = sy * cr * cp - cy * sr * sp

    return [w, x, y, z]



class pickUp():
    def __init__(self, vehicle, setpoint_pub, type_pub, pickupPos):

        # Drone
        self.vehicle = vehicle
        self.setpoint_pub = setpoint_pub
        self.step = "GO_TO_PICKUP" # "GO_TO_PICKUP", "PRECLAND", "DISARM"
        #self.actuator = actuator
        # Aruco
        self.arucoPose = None
        self.arucoAngle = None
        self.type_pub = type_pub

        try:
            print("\nCreating sky_vision aruco detector subscribers...")
            rospy.Subscriber('/sky_vision/down_cam/aruco/pose', Point, self.aruco_pose_callback)
            rospy.Subscriber('/sky_vision/down_cam/aruco/angle', Vector3, self.aruco_angle_callback)
            print("Subscribers are up!")
        except:
            print("Error!")

        self.aruco_count = 0
        
        # Pickup position
        self.pickupPos = PoseStamped()
        self.pickupPos.pose.position = pickupPos

    
    def intStateMachine(self):

        # Init Pickup phase (looking for arucos)
        self.type_pub.publish(String("aruco"))

        if self.step == "GO_TO_PICKUP":
            if self.arucoPose is None:
                self.setpoint_pub.publish(self.pickupPos)
                self.aruco_count = 0
            else:
                self.aruco_count += 1

                if self.aruco_count > PICKUP_ROBUST_PRECLAND:
                    print("\nAruco base was found! Starting precision landing...")
                    
                    # Init Precision Landing
                    if self.vehicle.mode != 'LAND':
                        self.vehicle.mode = dronekit.VehicleMode('LAND')
                        while self.vehicle.mode != 'LAND':
                            time.sleep(1)
                        print('\nVehicle in LAND mode')
                    
                    self.aruco_count = 0
                    self.step = "PRECLAND"

        elif self.step == "PRECLAND":
            self.precland()
            if self.vehicle.armed == False:
                print("\nPrecision landing done!")
                self.step = "DISARM"

        elif self.step == "DISARM":
            # self.vehicle.armed = False
            # while self.vehicle.armed:
            #     time.sleep(1)
            # print('Vehicle disarmed!')
            #self.actuator.open()
            if self.vehicle.mode != 'GUIDED':
                self.vehicle.mode = dronekit.VehicleMode('GUIDED')
                while self.vehicle.mode != 'GUIDED':
                    time.sleep(1)
                print('\nVehicle in GUIDED mode, waiting to be armed...')

            # Finish the pickup phase
            return "TAKEOFF"
        
        return "PICKUP"
    
    def precland(self, time=0):
        dist = float(self.arucoPose[2])/100
        msg = self.vehicle.message_factory.landing_target_encode(
            time,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            self.arucoAngle[0],
            self.arucoAngle[1],
            dist,
            0,
            0,
        )
        self.vehicle.send_mavlink(msg)

    def aruco_pose_callback(self, msg):
        try:
            self.arucoPose = [msg.x, msg.y, msg.z]
        except:
            self.arucoPose = None

    def aruco_angle_callback(self, msg):
        try:
            self.arucoAngle = [msg.x, msg.y, msg.z]
        except:
            self.arucoAngle = None


class transitZone():
    def __init__(self, setpoint_pub, setpoint_pub_raw, type_pub, type_pub_front,  transitPos):

        # Drone
        self.setpoint_pub = setpoint_pub
        self.setpoint_pub_raw = setpoint_pub_raw
        self.type_pub = type_pub
        self.type_pub_front = type_pub_front
        self.step = "GO_TO_START"
        self.goal_pose = PoseStamped()
        
        # Line
        self.line_error = None
        self.line_angle = None
        self.line_detected = 0

        try:
            print("\nCreating sky_vision line detector subscriber...")
            rospy.Subscriber('/sky_vision/down_cam/line/pose', Point, self.line_pose_callback)
            print("Subscriber is up!")
            print("\nCreating sky_vision window detector subscriber...")
            rospy.Subscriber('/sky_vision/front_cam/window/pose', Point, self.window_pose_callback)
            print("Subscriber is up!")

        except:
            print("Error!")

        self.line_count = 0

        self.step =  "GO_TO_START"# "FOLLOW_LINE", "WINDOW"

        # Pickup position
        #self.transitPos = PositionTarget()
        self.transitPos = PoseStamped()
        self.transitPos.pose.position = transitPos

        # Controller parameters
        self.Kp = 0.112                 # Ku=0.14 T=6. PID: p=0.084,i=0.028,d=0.063. PD: p=0.112, d=0.084/1. P: p=0.07
        self.Ki = 0
        self.kd = 1
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.Kp_ang = 0.01             # Ku=0.04 T=2. PID: p=0.024,i=0.024,d=0.006. PD: p=0.032, d=0.008. P: p=0.02/0.01
        self.Ki_ang = 0
        self.kd_ang = 0
        self.integral_ang = 0
        self.derivative_ang = 0
        self.last_ang = 0

        self.forward_velocity = 0.3

        # Window parameters
        self.window_maxd = 3
        self.windowDetected = False
        self.window_y_vel = 0.1
        self.window_z_vel = 0.1
        self.window_move_vel = 0.1
        self.window_dy = None
        self.window_dz = None



    def line_pose_callback(self, msg):
        try:
            self.line_error = msg.x
            self.line_angle = msg.y
            self.line_detected = msg.z
        except:
            self.line_error = None
            self.line_angle = None
            self.line_detected = None

    def window_pose_callback(self, msg):
        #print(msg)
        try:
            self.window_dy = msg.y
            self.window_dz = msg.z
            #print("Callback: " ,self.window_dy)
        except:
            self.window_dy = None
            self.window_dz = None

    def intStateMachine(self):

        # Init Pickup phase (looking for arucos)
        self.type_pub.publish(String("line"))
        self.type_pub_front.publish(String("window"))

        if self.step == "GO_TO_START":
            # print error
            if self.line_detected == 0:
                #self.setpoint_pub_raw.publish(self.transitPos)
                self.go_to_local(0, -2, ALTITUDE, yaw=math.pi/2 * (-1), sleep_time=10)
                self.line_count = 0
            else:
                self.line_count += 1
                print("line count:",self.line_count)

                if self.line_count > TRANSIT_ROBUST_LINEFOLLOWER:
                    
                    # Init Line Follower
                    print("\nLine track found, starting line follower...")
                    self.line_count = 0
                    self.step = "FOLLOW_LINE"

        elif self.step == "FOLLOW_LINE":
            if self.line_detected == 1:
                self.follow_line()
            # elif self.windowDetected:
            #      self.step = "WINDOW"
            elif dronePos.y < -7:
                return "DROP"
        
        elif self.step == "WINDOW":
            print("Detecting window")
            self.window_detetion()
            if self.window_dy is None and self.window_dz is None:
                self.windowDetected = False
                self.step = "FOLLOW_LINE"

        return "TRANSIT"

    def intStateMachine(self):
            self.type_pub.publish(String("Go_back"))
            if self.step == "GO_TO_START":
                print("Going to transit")
                self.go_to_local(0, -2, GO_BACK_1[2], yaw=math.pi/2 * (-1), sleep_time=10)
                return "DROP"
                

            return "TRANSIT"


    def go_to_local(self, goal_x, goal_y, goal_z, yaw = None, sleep_time=5):
        # rospy.loginfo("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))

        init_time = now = time.time()
        while not rospy.is_shutdown() and now-init_time < sleep_time:
            self.set_position(goal_x, goal_y, goal_z, yaw)
            now = time.time()
        
        # rospy.loginfo("Arrived at requested position")
    
    def set_position(self, x, y, z, yaw = None):
        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)
        #self.goal_pose.header.frame_id = ""
        # if yaw == None:
        #     self.goal_pose.pose.orientation = self.drone_pose.pose.orientation
        # else:
        [self.goal_pose.pose.orientation.x, 
        self.goal_pose.pose.orientation.y, 
        self.goal_pose.pose.orientation.z,
        self.goal_pose.pose.orientation.w] = quaternion_from_euler(0,0,yaw) #roll,pitch,yaw
            #print("X: " + str(self.goal_pose)))

        self.setpoint_pub.publish(self.goal_pose)

class dropZone():
    def __init__(self, vehicle, setpoint_pub, type_pub, pickupPos):
        # Drone
        self.vehicle = vehicle
        self.arucoPose = None
        self.arucoAngle = None
        self.setpoint_pub = setpoint_pub
        self.type_pub = type_pub
        self.pickupPos = pickupPos
        #self.actuator = actuator
        self.step = "GO_TO_ZONE" # "GO_TO_ZONE", "CENTER_YAW", "GO_TO_DROP", "DROP", "GO_UP", "END"
        self.goal_pose = PoseStamped()

        rospy.Subscriber('/sky_vision/down_cam/aruco/pose', Point, self.aruco_pose_callback)
        rospy.Subscriber('/sky_vision/down_cam/aruco/angle', Vector3, self.aruco_angle_callback)
        
        self.desHeight = 1
        self.blockHeight = 0.5
        self.blocknum = 1

    def precdrop(self ,dronePos , timer=0):
        if self.arucoPose is not None and self.arucoAngle is not None:
            if dronePos.z <= 0.1 + self.blockHeight*self.blocknum:
                self.vehicle.mode = dronekit.VehicleMode('GUIDED')
                while self.vehicle.mode != 'GUIDED':
                    time.sleep(1)
                self.step = "GO_UP"
                print('vehicle in GUIDED mode')
            else:    
                dist = float(self.arucoPose[2])/100 - (0.1 +self.blockHeight*self.blocknum)
                msg = self.vehicle.message_factory.landing_target_encode(
                    timer,
                    0,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    self.arucoAngle[0],
                    self.arucoAngle[1],
                    dist,
                    0,
                    0,
                    )
        
                self.vehicle.send_mavlink(msg)
                print("Mensagem enviada")
            
    def center_yaw(self, droneOri, dronePos):
        if math.sqrt(droneOri[2]**2) <= math.pi - 0.2:
            self.go_to_local(dronePos.x, dronePos.y, dronePos.z, yaw=math.pi, sleep_time=10)
        else:
            self.step = "GO_TO_DROP"

    def go_up(self, dronePos):
        if dronePos.z <= self.desHeight:
            self.go_to_local(dronePos.x, dronePos.y, self.desHeight, math.pi/2 * (-1),  sleep_time=10)
        else:
            self.step = "END"

            
    def intStateMachine(self):
        self.type_pub.publish(String("arucrooked"))
        if self.step == "GO_TO_ZONE":
            print("Going to zone")
            self.go_to_local(self.pickupPos.x, self.pickupPos.y, self.desHeight, yaw=math.pi/2 * (-1), sleep_time=10)
            return "GO_BACK"
        return "DROP"
    

    


    def go_to_local(self, goal_x, goal_y, goal_z, yaw = None, sleep_time=5):
        # rospy.loginfo("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))

        init_time = now = time.time()
        while now-init_time < sleep_time:
            self.set_position(goal_x, goal_y, goal_z, yaw)
            now = time.time()
        
        # rospy.loginfo("Arrived at requested position")
    
    def set_position(self, x, y, z, yaw = None):
        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)
        #self.goal_pose.header.frame_id = ""
        # if yaw == None:
        #     self.goal_pose.pose.orientation = self.drone_pose.pose.orientation
        # else:
        [self.goal_pose.pose.orientation.x, 
        self.goal_pose.pose.orientation.y, 
        self.goal_pose.pose.orientation.z,
        self.goal_pose.pose.orientation.w] = quaternion_from_euler(0,0,yaw) #roll,pitch,yaw
            #print("X: " + str(self.goal_pose)))

        self.setpoint_pub.publish(self.goal_pose)
        
    def aruco_pose_callback(self, msg):
        try:
            self.arucoPose = [msg.x, msg.y, msg.z]
            
        except:
            self.arucoPose = None
    def aruco_angle_callback(self, msg):
        try:
            self.arucoAngle = [msg.x, msg.y, msg.z]
        except:
            self.arucoAngle = None

class goBack():
    def __init__(self, vehicle, setpoint_pub, type_pub ):
        # Drone
        self.vehicle = vehicle
        self.setpoint_pub = setpoint_pub
        self.type_pub = type_pub
        #self.actuator = actuator
        self.goal_pose = PoseStamped()

        self.step = "GO_BACK_1" # "GO_BACK_2"

    
    def intStateMachine(self):
        print("Machine do go back")
        self.type_pub.publish(String("Go_back"))
        if self.step == "GO_BACK_1":
            print("Going back to step 1")
            self.go_to_local(GO_BACK_1[0], GO_BACK_1[1], GO_BACK_1[2], yaw=math.pi/2 * (-1), sleep_time=10)
            self.step = "GO_BACK_2"
        elif self.step == "GO_BACK_2":
            print("Going back to step 2 ")
            self.go_to_local(GO_BACK_2[0], GO_BACK_2[1], GO_BACK_2[2], yaw=math.pi/2 * (-1), sleep_time=10)
            self.step = "GO_BACK_3"
        elif self.step == "GO_BACK_3":
            print("Going back to step 3 ")
            self.go_to_local(PICKUP_POINT[0], PICKUP_POINT[1], PICKUP_POINT[2], yaw=math.pi/2 * (-1), sleep_time=10)
            return "PICKUP"
            

        return "GO_BACK"
        


    def go_to_local(self, goal_x, goal_y, goal_z, yaw = None, sleep_time=5):
        # rospy.loginfo("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))

        init_time = now = time.time()
        while now-init_time < sleep_time:
            self.set_position(goal_x, goal_y, goal_z, yaw)
            now = time.time()
        
        # rospy.loginfo("Arrived at requested position")
    
    def set_position(self, x, y, z, yaw = None):
        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)
        #self.goal_pose.header.frame_id = ""
        # if yaw == None:
        #     self.goal_pose.pose.orientation = self.drone_pose.pose.orientation
        # else:
        [self.goal_pose.pose.orientation.x, 
        self.goal_pose.pose.orientation.y, 
        self.goal_pose.pose.orientation.z,
        self.goal_pose.pose.orientation.w] = quaternion_from_euler(0,0,yaw) #roll,pitch,yaw
            #print("X: " + str(self.goal_pose)))

        self.setpoint_pub.publish(self.goal_pose)

class drone(): # Unifies all drone movement elements, including main state machine
    def __init__(self):

        # Dronekit vehicle init
        self.vehicle = dronekit.connect("tcp:127.0.0.1:5763", baud=57600)


        #self.actuator = Actuator()
        # Mavros Publishers 
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.pub_ang_vel = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

        self.setpoint_pub_raw = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        self.type_pub = rospy.Publisher('/sky_vision/down_cam/type', String, queue_size=1) # Sends detection type to vision node
        self.type_pub_front = rospy.Publisher('/sky_vision/front_cam/type', String, queue_size=1) # Sends detection type to vision node
        # Mavros Subscribers
        try:
            print("\nCreating mavros local position subscriber...")
            rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)
            print("Subscriber is up!")
        except:
            print("Error")

        # Drone
        self.dronePos = None
        self.droneOri = None
        self.targetZ = ALTITUDE
        self.curStep = "TAKEOFF" # TAKEOFF, PICKUP, TRANSIT, DROP
        self.hasBlock = False # Set to True to skip prec land, testing purposes only
        self.takeoffNext = None #What step the drone should go to after takeoff, Debugging purposes only

        self.goal_pose = PoseStamped()

        # PICKUP ZONE
        pickupPoint = Point()
        pickupPoint.x = PICKUP_POINT[0]
        pickupPoint.y = PICKUP_POINT[1]
        pickupPoint.z = PICKUP_POINT[2]

        self.pickup = pickUp(self.vehicle, self.setpoint_pub, self.type_pub, pickupPoint)

        # TRANSIT ZONE
        transitPoint = Point()
        transitPoint.x = TRANSIT_POINT[0]
        transitPoint.y = TRANSIT_POINT[1]
        transitPoint.z = TRANSIT_POINT[2]

        self.transit = transitZone(self.setpoint_pub, self.setpoint_pub_raw, self.type_pub,self.type_pub_front,  transitPoint)

        # DROP ZONE
        dropPoint = Point()
        dropPoint.x = DROP_POINT[0]
        dropPoint.y = DROP_POINT[1]
        dropPoint.z = DROP_POINT[2]

        self.drop = dropZone(self.vehicle, self.setpoint_pub, self.type_pub, dropPoint)

        # GO BACK
        self.go_back = goBack(self.vehicle, self.setpoint_pub, self.type_pub)

        print("\nWaiting to be armed...")


    # General mission state machine
    def stateMachine(self):
        # print(self.curStep)

        if self.curStep == "TAKEOFF":
            print("TAKEOFF")
            if self.vehicle.armed == True:

                self.vehicle.simple_takeoff(self.targetZ)
                time.sleep(0.5)

                if self.dronePos.z >= self.targetZ*0.50:
                    if self.takeoffNext is not None:
                        self.curStep = self.takeoffNextPICKUP
                    elif self.hasBlock is False:
                        self.curStep = "PICKUP"
                    else:
                        self.curStep = "TRANSIT"
                    print("\nTakeoff done!")

        elif self.curStep == "PICKUP":
            print("PICKUP")
            self.curStep = self.pickup.intStateMachine()
            if self.curStep == "TAKEOFF":
                self.hasBlock = True

        elif self.curStep == "TRANSIT":
            print("TRANSIT")
            self.curStep = self.transit.intStateMachine()
            time.sleep(5)
        #print(self.curStep)
        elif self.curStep == "DROP":
            print("DROP")
            self.type_pub_front.publish(String(""))
            self.curStep = self.drop.intStateMachine()
            time.sleep(5)
        elif self.curStep == "GO_BACK":
            print("GO BACK")
            self.type_pub_front.publish(String(""))
            self.curStep = self.go_back.intStateMachine()
            
            self.hasBlock = False

    def dronePosCallback(self, data):
        try:
            self.dronePos = data.pose.position
            quaternions = data.pose.orientation
            self.droneOri = quaternions_to_euler_angle(quaternions.x, quaternions.y, quaternions.z, quaternions.w)
            # print(self.dronePos)
        except:
            pass


if __name__ == '__main__':

    rospy.init_node('mainSim', anonymous=True)

    indoor = drone()

    time.sleep(5)

    while rospy.is_shutdown() is False:
        try:
            indoor.stateMachine()
            
        except KeyboardInterrupt:
            print("Shutting down")
            break