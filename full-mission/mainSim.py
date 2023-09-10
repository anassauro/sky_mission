import rospy
import time
import math
import dronekit
from pymavlink import mavutil

from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Vector3
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget


# (x, y, z) pickup estimate location
PICKUP_POINT = [0, 1, 2]

# (x, y, z) transit estimate location
TRANSIT_POINT = [2, -5, 2]

# (x, y, z) pickup estimate location
DROP_POINT = [8, 0, 2]

# Number os arucos detected to avoid false positives
PICKUP_ROBUST_PRECLAND = 5

# Number os lines detected to avoid false positives
TRANSIT_ROBUST_LINEFOLLOWER = 200


class pickUp():
    def __init__(self, vehicle, setpoint_pub, type_pub, pickupPos):

        # Drone
        self.vehicle = vehicle
        self.setpoint_pub = setpoint_pub
        self.step = "GO_TO_PICKUP" # "GO_TO_PICKUP", "PRECLAND", "DISARM"

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
    def __init__(self, setpoint_pub, setpoint_pub_raw, type_pub, transitPos):

        # Drone
        self.setpoint_pub = setpoint_pub
        self.setpoint_pub_raw = setpoint_pub_raw
        self.type_pub = type_pub
        self.step = "GO_TO_START"
        
        # Line
        self.line_error = None
        self.line_angle = None

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

        self.step = "GO_TO_START"# "GO_TO_START", "FOLLOW_LINE", "WINDOW"

        # Pickup position
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

        self.forward_velocity = 0.1

        # Window parameters
        self.window_maxd = 3
        self.windowDetected = False
        self.window_y_vel = 0.1
        self.window_z_vel = 0.1
        self.window_move_vel = 0.1



    def line_pose_callback(self, msg):
        try:
            self.line_error = msg.x
            self.line_angle = msg.y
        except:
            self.line_error = None
            self.line_angle = None

    def window_pose_callback(self, msg):
        try:
            self.window_dy = msg.y
            self.window_dz = msg.Z
        except:
            self.window_dy = None
            self.window_dz = None

    def intStateMachine(self):

        # Init Pickup phase (looking for arucos)
        self.type_pub.publish(String("line"))

        if self.step == "GO_TO_START":
            if self.line_error is None and self.line_angle is None:
                self.setpoint_pub.publish(self.transitPos)
                self.line_count = 0
            else:
                self.line_count += 1

                if self.line_count > TRANSIT_ROBUST_LINEFOLLOWER:
                    
                    # Init Line Follower
                    print("\nLine track found, starting line follower...")
                    self.line_count = 0
                    self.step = "FOLLOW_LINE"

        elif self.step == "FOLLOW_LINE":
            if self.line_error is not None and self.line_angle is not None:
                self.follow_line()
        
        elif self.step == "WINDOW":
            self.window_detetion()

        return "TRANSIT"
    
    def follow_line(self):
        error_corr = -1 * (self.Kp * self.line_error + self.Ki * self.integral + self.kd * self.derivative)  # PID controler
        # print("error_corr:  ", error_corr, "\nP", normal_error * self.Kp, "\nI", self.integral* self.Ki, "\nD", self.kd * self.derivative)

        angle = int(self.line_angle)

        self.integral_ang = float(self.integral_ang + angle)
        self.derivative_ang = angle - self.last_ang
        self.last_ang = angle

        ang_corr = -1 * (self.Kp_ang * angle + self.Ki_ang * self.integral_ang + self.kd_ang * self.derivative_ang)  # PID controler

        setpoint_ = PositionTarget()
        vel_setpoint_ = Vector3()

        vel_setpoint_.x = self.forward_velocity
        vel_setpoint_.y = error_corr

        if abs(vel_setpoint_.y) > vel_setpoint_.x/2:
            vel_setpoint_.y = vel_setpoint_.x/2 * (vel_setpoint_.y / abs(vel_setpoint_.y))

        yaw_setpoint_ = ang_corr * math.pi / 180 * 5

        print(f"x: {vel_setpoint_.x} | y: {vel_setpoint_.y} | yaw: {yaw_setpoint_}")

        setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
        
        setpoint_.velocity.x = vel_setpoint_.x
        setpoint_.velocity.y = vel_setpoint_.y
        setpoint_.velocity.z = 0

        setpoint_.yaw = yaw_setpoint_

        self.setpoint_pub_raw.publish(setpoint_)

    
    def window_detetion(self):
        # Centralizing in the window
        if not self.windowDetected:
            if abs(self.window_dy) < self.maxd:
                self.window_y_vel = 0
                self.windowDetected = True
            else:
                if self.window_dy > 0:
                    self.window_y_vel = -0.1
                else:
                    self.window_y_vel = 0.1

            # Set velocity 
            setpoint_ = PositionTarget()
            vel_setpoint_ = Vector3()

            vel_setpoint_.x = 0
            vel_setpoint_.y = self.window_y_vel
            vel_setpoint_.z = 0

            yaw_setpoint_ = 0
            # Print velocity
            print(f"x: {vel_setpoint_.x} | y: {vel_setpoint_.y} | z: {vel_setpoint_.z}")

            setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
            
            
            setpoint_.velocity.x = vel_setpoint_.x
            setpoint_.velocity.y = vel_setpoint_.y
            setpoint_.velocity.z = vel_setpoint_.z

            setpoint_.yaw = yaw_setpoint_

            
            self.setpoint_pub_raw.publish(setpoint_)

        # Passing through the window
        else:
            #  Move foward
            setpoint_ = PositionTarget()
            vel_setpoint_ = Vector3()

            vel_setpoint_.x = self.window_move_vel
            vel_setpoint_.y = 0
            vel_setpoint_.z = 0
            yaw_setpoint_ = 0
            setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
            
            setpoint_.velocity.x = vel_setpoint_.x
            setpoint_.velocity.y = vel_setpoint_.y
            setpoint_.velocity.z = vel_setpoint_.z

            setpoint_.yaw = yaw_setpoint_

            self.setpoint_pub_raw.publish(setpoint_)




class dropZone():
    def __init__(self, pub_vel, type_pub):
        self.pub_vel = pub_vel
        self.type_pub = type_pub
        self.blockNum = 0
        self.blockHeight = 0.5
        self.step = 1
        self.targetCenter = None
        self.tol = 5
        rospy.Subscriber('/sky_vision/down_cam/block/pose', Point, self.center_callback)
    
    def getError(self, center, imShape):
        if center is not None:
            errorX = center[0] - imShape[1]//2
            errorY = -center[1] + imShape[0]//2
            if abs(errorX) < self.tol and abs(errorY) < self.tol and self.step != 0:
                self.step = 2
            elif self.step != 0:
                self.step = 1
            return [errorX, errorY]
    
    def centralize(self, error, dronePos):
        self.tol = 30/(dronePos.z - self.blockHeight*self.blockNum)
        if error is not None:
            
            vel = TwistStamped()
            vel.twist.linear.x = error[0]*(dronePos.z - self.blockHeight*self.blockNum)/1500
            vel.twist.linear.y = error[1]*(dronePos.z - self.blockHeight*self.blockNum)/1500
            
            self.pub_vel.publish(vel)
            return True
        else:
            
            self.pub_vel.publish(TwistStamped())
            return False
        
    def lowerBlock(self, dronePos):
        if dronePos.z > self.blockHeight*self.blockNum + 0.1 and self.blockNum != 0:
            
            vel = TwistStamped()
            vel.twist.linear.z = -0.1*(dronePos.z - self.blockHeight*self.blockNum)
            self.pub_vel.publish(vel)
            return True
        elif dronePos.z > 0.4 and self.blockNum == 0:
            vel = TwistStamped()
            vel.twist.linear.z = -0.1*(dronePos.z - 0.4)
            self.pub_vel.publish(vel)
            return True
        else:
            print("block lowered")
            self.pub_vel.publish(TwistStamped())
            self.blockNum += 1
            self.step = 3
            
            return False
        
    def goUp(self, dronePos):
        if dronePos.z < 1:
            vel = TwistStamped()
            vel.twist.linear.z = 0.2
            self.pub_vel.publish(vel)
            return True
        else:
            self.pub_vel.publish(TwistStamped())
            self.step = 0
            return False
        
    #Internal State Machine
    def stackBlock(self, dronePos, center, imShape):
        self.type_pub.publish(String("block"))
        if self.blockNum > 0:
            error = self.getError(center, imShape)
            if self.step == 1:
                print("centralizing")
                self.centralize(error, dronePos)
            elif self.step == 2:
                print("lowering")
                self.lowerBlock(dronePos) 
            elif self.step == 3:
                print("going up")
                self.goUp(dronePos)
            elif self.step == 0:
                print("Stop drop")
                self.step = 1
                return "PICKUP"
            
        elif self.blockNum == 0:
            error = None
            if self.step == 1:
                print("centralizing")
                self.centralize(error)
            elif self.step == 2:
                print("lowering")
                self.lowerBlock() 
            elif self.step == 3:
                print("going up")
                self.goUp()
            elif self.step == 0:
                print("Stop drop")
                self.step = 1
                return "PICKUP"

        return "DROP"
    
    def center_callback(self, msg):
        try:
            self.targetCenter = [msg.x, msg.y]
        except:
            self.targetCenter = None
        return
    

class drone(): # Unifies all drone movement elements, including main state machine
    def __init__(self):

        # Dronekit vehicle init
        self.vehicle = dronekit.connect("tcp:127.0.0.1:5763", baud=57600)

        # Mavros Publishers 
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.pub_ang_vel = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.setpoint_pub_raw = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        self.type_pub = rospy.Publisher('/sky_vision/down_cam/type', String, queue_size=1) # Sends detection type to vision node

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
        self.targetZ = 2
        self.curStep = "TAKEOFF" # TAKEOFF, PICKUP, TRANSIT, DROP
        self.hasBlock = False

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

        self.transit = transitZone(self.setpoint_pub, self.setpoint_pub_raw, self.type_pub, transitPoint)

        # DROP ZONE
        dropPoint = Point()
        dropPoint.x = DROP_POINT[0]
        dropPoint.y = DROP_POINT[1]
        dropPoint.z = DROP_POINT[2]

        self.drop = dropZone(self.pub_vel, self.type_pub)

        print("\nWaiting to be armed...")


    # General mission state machine
    def stateMachine(self):

        if self.curStep == "TAKEOFF":
            if self.vehicle.armed == True:

                self.vehicle.simple_takeoff(self.targetZ)
                time.sleep(0.5)

                if self.dronePos.z >= self.targetZ*0.50:
                    if self.hasBlock is False:
                        self.curStep = "PICKUP"
                    else:
                        self.curStep = "TRANSIT"
                    print("\nTakeoff done!")

        elif self.curStep == "PICKUP":
            self.curStep = self.pickup.intStateMachine()
            if self.curStep == "TAKEOFF":
                self.hasBlock = True

        elif self.curStep == "TRANSIT":
            self.curStep = self.transit.intStateMachine()
        #print(self.curStep)
            
    def dronePosCallback(self, data):
        try:
            self.dronePos = data.pose.position
            self.droneOri = data.pose.orientation
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