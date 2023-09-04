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
from std_msgs.msg import Int16




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
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.pub_ang_vel = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)
        self.dronePos = None
        self.droneOri = None
        self.curStep = 0 # 0 = takeoff, 1 = precision landing, 2 = line following, 4 = passing through gate, 5 = block stacking

    def dronePosCallback(self, data):
        try:
            self.drone_pos = data.pose.position
            self.drone_ori = data.pose.orientation
        except:
            pass
    


if __name__ == '__main__':
    rospy.init_node('mainSim', anonymous=True)
    vis = visual()
    time.sleep(5)
    while rospy.is_shutdown() is False:
        try:
            cv.imshow("front", vis.front_image)
            cv.imshow("down", vis.down_image)
            cv.waitKey(1) & 0xFF
        except KeyboardInterrupt:
            print("Shutting down")
            cv.destroyAllWindows()
            break