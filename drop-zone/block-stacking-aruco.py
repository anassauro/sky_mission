
import rospy
import numpy as np
import time
import math
import dronekit
from pymavlink import mavutil

from geometry_msgs.msg import PoseStamped, Point, Vector3
from std_msgs.msg import String


class dropZone():
    def __init__(self, vehicle):
        # Drone
        self.vehicle = vehicle
        self.arucoPose = None
        self.arucoAngle = None
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.type_pub = rospy.Publisher('/sky_vision/down_cam/type', String, queue_size=1)
        self.step = "CENTER_YAW" # "CENTER_YAW", "GO_TO_DROP", "DROP", "GO_UP", "END"
        
        rospy.Subscriber('/sky_vision/down_cam/aruco/pose', Point, self.aruco_pose_callback)
        rospy.Subscriber('/sky_vision/down_cam/aruco/angle', Vector3, self.aruco_angle_callback)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)
        self.desHeight = 5
        self.blockHeight = 0.5
        self.blocknum = 1
        self.dronePos = None
        self.droneOri = None

    def precdrop(self , timer=0):
        if self.arucoPose is not None and self.arucoAngle is not None:
            if self.dronePos[2] <= 0.1 + self.blockHeight*self.blocknum:
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
    def center_yaw(self):
        if math.sqrt(self.droneOri**2) <= math.pi - 0.2:
            print(self.droneOri)
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = self.dronePos[0]
            msg.pose.position.y = self.dronePos[1]
            msg.pose.position.z = self.dronePos[2]
            self.setpoint_pub.publish(msg)
            time.sleep(0.25)
        else:
            self.step = "GO_TO_DROP"

    def go_up(self):
        if self.dronePos[2] <= self.desHeight:
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = self.dronePos[0]
            msg.pose.position.y = self.dronePos[1]
            msg.pose.position.z = self.desHeight
            self.setpoint_pub.publish(msg)
            time.sleep(0.25)
        else:
            self.step = "END"

            
    def intStateMachine(self):
        self.type_pub.publish(String("arucrooked"))
        if self.step == "CENTER_YAW":
            print("Centering yaw")
            self.center_yaw()
        elif self.step == "GO_TO_DROP":
            print("Going to drop")
            self.vehicle.mode = dronekit.VehicleMode('LAND')
            while self.vehicle.mode != 'LAND':
                time.sleep(1)
            self.precdrop()
        elif self.step == "DROP":
            print("Dropping")
            time.sleep(5)
            self.step = "GO_UP"
        elif self.step == "GO_UP":
            print("Going up")
            self.go_up()
        elif self.step == "END":
            print("END")

        
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
    def dronePosCallback(self, msg):
        try:
            self.dronePos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            quaternions = msg.pose.orientation
            self.droneOri = math.atan2(2*(quaternions.w*quaternions.z + quaternions.x*quaternions.y), 1 - 2*(quaternions.w**2 + quaternions.x**2))
        except:
            pass
    
if __name__ == '__main__':
    rospy.init_node('dropZone', anonymous=True)
    vehicle = dronekit.connect("tcp:127.0.0.1:5763", baud=57600)
    pickup = dropZone(vehicle)
    time.sleep(5)
    while not rospy.is_shutdown():
        try:
            pickup.intStateMachine()
        except KeyboardInterrupt:
            break
