import rospy
import cv2 as cv
import numpy as np
import time
#import dronekit

from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, TransformStamped
from mavros_msgs.msg import PositionTarget 
from std_msgs.msg import Int16
from tf2_msgs.msg import TFMessage

class blockMarker():
    def __init__(self):
        self.center = None
        self.cv_image = None
        
    
    def findMask(self):
        hsv = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)
        lower = np.array([90,50,50])
        upper = np.array([105, 255, 255])
        mask = cv.inRange(hsv, lower, upper)
        mask = cv.erode(mask, None, iterations=2)
        return mask

    def mapCircle(self):
        mask = self.findMask()
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        maxArea = 0
        maxContour = None
        cx, cy = 0, 0
        for i in contours:
            curve = cv.approxPolyDP(i, 0.01*cv.arcLength(i, True), True)
            area = cv.contourArea(curve)
            if area > maxArea:
                maxArea = area
                maxContour = curve
        if maxContour is not None:
            rect = cv.minAreaRect(maxContour)
            (cx, cy), (w, h), angle = rect

        return [cx, cy]
    
    def update(self):
        capture = cv.VideoCapture(0)
        ret, self.cv_image = capture.read()
        self.center = self.mapCircle()
        print(self.center)
        return


class blockStacker():
    def __init__(self):
        
        #self.vehicle = vehicle
        self.marker = blockMarker()

        rospy.init_node('blockStacking', anonymous=False)

        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.pub_ang_vel = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        rospy.Subscriber('tf', TFMessage, self.dronePosCallback)
        
        self.dronePos = None
        self.droneOri = None

        self.blockNum = 1
        self.blockHeight = 0.5
       
        self.step = 1
        self.stacking = True
        self.targetCenter = None
        self.pckgCenter = None
        self.tol = 5

    def updateCenters(self):
        """
        if self.marker.center is not None and len(self.marker.centers) == 2:
            self.TargetCenter = self.marker.centers[1]
            self.pckgCenter = self.marker.centers[0]
        elif self.marker.centers is not None and len(self.marker.centers) == 1:
            self.TargetCenter = None
            self.pckgCenter = self.marker.centers[0]
        else:
            self.TargetCenter = None
            self.pckgCenter = None
        """
        if self.marker.center is not None:
            self.TargetCenter = self.marker.center
        else:
            self.TargetCenter = None
    def getError(self):
        self.updateCenters()
        if self.TargetCenter is not None:
            errorX = self.TargetCenter[0] - self.marker.cv_image.shape[1]//2
            errorY = -self.TargetCenter[1] + self.marker.cv_image.shape[0]//2
            if abs(errorX) < self.tol and abs(errorY) < self.tol and self.step != 0:
                self.step = 2
            elif self.step != 0:
                self.step = 1
            return [errorX, errorY]
        #elif self.TargetCenter is not None and self.dronePos.z <= 0.5 + self.blockHeight*self.blockNum:
        #    errorX = self.TargetCenter[0] - self.pckgCenter[0]
        #    errorY = -self.TargetCenter[1] + self.pckgCenter[1]
        #    if abs(errorX) < self.tol and abs(errorY) < self.tol and self.step != 0:
        #        self.step = 2
        #    elif self.step != 0:
        #        self.step = 1
        #    return [errorX, errorY]
        #elif self.pckgCenter is not None:
        #    errorX = 0
        #    errorY = 0
        #    self.step = 2
        #    return [errorX, errorY]
        
        else:
            return None
    
    def centralize(self, error):
        if error is not None:
            
            vel = TwistStamped()
            vel.twist.linear.x = error[0]*(self.dronePos.z - self.blockHeight*self.blockNum)/3000
            vel.twist.linear.y = error[1]*(self.dronePos.z - self.blockHeight*self.blockNum)/3000
            
            self.pub_vel.publish(vel)
        else:
            self.centralizing = False
            self.pub_vel.publish(TwistStamped())
            self.pub_ang_vel.publish(TwistStamped())
        return
    
    def lowerBlock(self):
        if self.dronePos.z > self.blockHeight*self.blockNum + 0.1:
            
            vel = TwistStamped()
            vel.twist.linear.z = -0.05*(self.dronePos.z - self.blockHeight*self.blockNum)
            self.pub_vel.publish(vel)
            self.tol = 30/self.dronePos.z
        else:
            print("block lowered")
            self.pub_vel.publish(TwistStamped())
            self.blockNum += 1
            self.step = 3
            self.stacking = False
        return
    
    def goUp(self):
        if self.dronePos.z < 1:
            vel = TwistStamped()
            vel.twist.linear.z = 0.2
            self.pub_vel.publish(vel)
        else:
            self.pub_vel.publish(TwistStamped())
            self.step = 0
        return
    
    def stackBlock(self):
        if self.stacking:
            error = self.getError()
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
            print("Stop")

        time.sleep(0.5)
        return
    
    
    def dronePosCallback(self, data):
        try:
            
            self.dronePos = data.transforms.transform.translation
            print(self.dronePos) 
        except:
            pass   
        return
            



if __name__ == '__main__':
    
    #vehicle = dronekit.connect("tcp:127.0.0.1:5763", baud=57600)
    bs = blockStacker()
    bm = blockMarker()
    time.sleep(1)
    
    
    
    while not rospy.is_shutdown():
        try:
            bm.update()
            bs.stackBlock()
        except KeyboardInterrupt:
            print("Shutting down")
            cv.destroyAllWindows()
        
        

        