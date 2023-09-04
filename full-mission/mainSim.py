import sys
import rospy
import cv2 as cv
import numpy as np
import time
import dronekit
from pymavlink import mavutil

from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
from mavros_msgs.msg import PositionTarget
from cv_bridge import CvBridge, CvBridgeError

class pickUp():
    def __init__(self, vehicle, setpoint_pub, pickupPos):
        # Drone
        self.vehicle = vehicle
        self.aruco = None
        self.setpoint_pub = setpoint_pub
        self.step = "GO_TO_PICKUP" # "GO_TO_PICKUP", "PRECLAND", "DISARM"
        self.pickupPos = PoseStamped()
        self.pickupPos.pose.position = pickupPos
    
    def intStateMachine(self):
        if self.step == "GO_TO_PICKUP":
            if self.aruco is None:
                self.setpoint_pub.publish(self.pickupPos)
            else:
                if self.vehicle.mode != 'LAND':
                    self.vehicle.mode = dronekit.VehicleMode('LAND')
                    while self.vehicle.mode != 'LAND':
                        time.sleep(1)
                    print('vehicle in LAND mode')
                self.step = "PRECLAND"
        elif self.step == "PRECLAND":
            self.precland(self.aruco)
        elif self.step == "DISARM":
            self.vehicle.armed = False
            while self.vehicle.armed:
                time.sleep(1)
            print('vehicle disarmed')
            return "TAKEOFF"
        return "PICKUP"
    def precland(self, aruco,time=0):
        msg = self.vehicle.message_factory.landing_target_encode(
            time,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            aruco[0],
            aruco[1],
            aruco[2],
            0,
            0,
        )
        self.vehicle.send_mavlink(msg)
        print("Mensagem enviada")
    

    


class dropZone():
    def __init__(self, pub_vel):
        self.pub_vel = pub_vel
        self.blockNum = 0
        self.blockHeight = 0.5
        self.step = 1
        self.targetCenter = None
        self.tol = 5
    
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
        return
    #Internal State Machine
    def stackBlock(self, dronePos, center, imShape):
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

    
class drone(): # Unifies all drone movement elements, including main state machine
    def __init__(self):
        self.vehicle = dronekit.connect("tcp:127.0.0.1:5763", baud=57600)
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.pub_ang_vel = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.drop = dropZone(self.pub_vel)
        pickupPoint = Point()
        pickupPoint.x = 1
        pickupPoint.y = 1
        pickupPoint.z = 1.5
        self.pickup = pickUp(self.vehicle, self.setpoint_pub, pickupPoint)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)
        self.dronePos = None
        self.droneOri = None
        self.targetZ = 1.5
        self.curStep = "TAKEOFF" # TAKEOFF, PICKUP, TRANSIT, DROP
        self.hasBlock = False
    def stateMachine(self):
        if self.curStep == "TAKEOFF":
            if self.vehicle.armed:
                self.vehicle.simple_takeoff(self.targetZ)
                time.sleep(0.5)
                if self.dronePos.z >= self.targetZ*0.50:
                    if self.hasBlock is False:
                        self.curStep = "PICKUP"
                    else:
                        self.curStep = "TRANSIT"
                    print("takeoff done")
        elif self.curStep == "PICKUP":
            self.curStep = self.pickup.intStateMachine()
        #print(self.curStep)


            
    def dronePosCallback(self, data):
        try:
            self.dronePos = data.pose.position
            self.droneOri = data.pose.orientation
            print(self.dronePos)
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
            cv.destroyAllWindows()
            break