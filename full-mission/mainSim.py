import sys
import rospy
import cv2 as cv
import numpy as np
import time
import dronekit

from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import PositionTarget
from cv_bridge import CvBridge, CvBridgeError



class blockMarker():
    def __init__(self):
        self.centers = None
        
    def findMask(self, image):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        lower = np.array([110,50,50])
        upper = np.array([130, 255, 255])
        mask = cv.inRange(hsv, lower, upper)
        return mask

    def mapCircles(self, image):
        mask = self.findMask(image)
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        centers = []
        for i in contours:
            curve = cv.approxPolyDP(i, 0.01*cv.arcLength(i, True), True)
            M = cv.moments(curve)
            cX = int(M['m10']/M['m00'])
            cY = int(M["m01"]/M["m00"])
            centers.append([cX, cY])
        return centers

class dropZone():
    def __init__(self, pub_vel):
        self.pub_vel = pub_vel
        self.blockNum = 1
        self.blockHeight = 0.5
        
        self.targetCenter = None
        self.pckgCenter = None
        self.tol = 5
    
    def getError(self, center, image):
        if center is not None:
            errorX = center[0] - image.shape[1]//2
            errorY = -center[1] + image.shape[0]//2
            if abs(errorX) < self.tol and abs(errorY) < self.tol and self.step != 0:
                self.step = 2
            elif self.step != 0:
                self.step = 1
            return [errorX, errorY]
    def centralize(self, error, dronePos):
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
        if dronePos.z > self.blockHeight*self.blockNum + 0.1:
            
            vel = TwistStamped()
            vel.twist.linear.z = -0.1*(dronePos.z - self.blockHeight*self.blockNum)
            self.pub_vel.publish(vel)
            self.tol = 30/dronePos.z
            return True
        else:
            print("block lowered")
            self.pub_vel.publish(TwistStamped())
            self.blockNum += 1
            
            
            return False
        
    def goUp(self, dronePos):
        if dronePos.z < 1:
            vel = TwistStamped()
            vel.twist.linear.z = 0.2
            self.pub_vel.publish(vel)
            return True
        else:
            self.pub_vel.publish(TwistStamped())
            return False
        return
class visual(): ## Unifies all visual elements
    def __init__(self):
        self.bm = blockMarker()
        self.bridge = CvBridge()
        self.front_image_sub = rospy.Subscriber('/front_cam/image_raw', Image, self.frontCallback)
        self.down_image_sub = rospy.Subscriber('/down_cam/image_raw', Image, self.downCallback)
        self.front_image = None
        self.down_image = None
    
    def frontCallback(self, data):
        try:
            self.front_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #cv.imshow("front", self.front_image)
        #cv.waitKey(1) & 0xFF
    
    def downCallback(self, data):
        try:
            self.down_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #cv.imshow("down", self.down_image)
        #cv.waitKey(1) & 0xFF
    
class drone(): # Unifies all drone movement elements, including main state machine
    def __init__(self):
        self.vis = visual()
        
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.pub_ang_vel = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.drop = dropZone(self.pub_vel)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)
        self.dronePos = None
        self.droneOri = None
        self.curStep = 0 # Mudar o sentido das etapas quando elas forem adicionadas

    def stateMachine(self):
        if self.curStep == 0: 
            centralizing = self.drop.centralize(self.drop.getError(self.drop.targetCenter, self.vis.down_image), self.dronePos)
            if centralizing is False:
                self.curStep = 1
        elif self.curStep == 1:
            lowering = self.drop.lowerBlock(self.dronePos)
            if lowering is False:
                self.curStep = 2
        elif self.curStep == 2:
            goingUp = self.drop.goUp(self.dronePos)
            if goingUp is False:
                self.curStep = 4
        elif self.curStep == 3:
            print("Cabou :)") # Mudar para o próximo passo
    def dronePosCallback(self, data):
        try:
            self.dronePos = data.pose.position
            self.droneOri = data.pose.orientation
            
        except:
            pass
    


if __name__ == '__main__':
    rospy.init_node('mainSim', anonymous=True)
    indoor = drone()
    time.sleep(1)
    while rospy.is_shutdown() is False:
        try:
            print(indoor.dronePos)
            
        except KeyboardInterrupt:
            print("Shutting down")
            cv.destroyAllWindows()
            break