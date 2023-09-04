import rospy 
import cv2
import numpy as np


from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import PositionTarget

# Activate or deactivate movement
MOVE = True

# Activate or deactivate imshow
IMSHOW = True

# Constants
LEFT = -1
RIGHT = 1
MIDDLE = 0

class motion_detection:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/webcam/image_raw', Image, self.callback)
        self.setpoint_pub_ = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.background = None

        # Object position and velocity
        self.last_object = None
        self.object_vel = None
        self.object_pos = None
        self.last_vels = [None, None, None]

        # Ready to pass through window
        self.go = False

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.go:
            self.move(cv_image)
        else:
            self.detect(cv_image)
    
    def move(self,cv_image):
        # Set a velocity to go foward
        setpoint_ = PositionTarget()
        vel_setpoint_ = Vector3()

        vel_setpoint_.x = 1
        vel_setpoint_.y = 0
        vel_setpoint_.z = 0
        yaw_setpoint_ = 0
        setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
        
        setpoint_.velocity.x = vel_setpoint_.x
        setpoint_.velocity.y = vel_setpoint_.y
        setpoint_.velocity.z = vel_setpoint_.z

        setpoint_.yaw = yaw_setpoint_

        if MOVE:
            self.setpoint_pub_.publish(setpoint_)

        if IMSHOW:
            cv2.imshow("Moving", cv_image)
            cv2.waitKey(1) & 0xFF

        

    def detect(self, frame):
        # frame to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Object mask
        lower_mask = np.array([ 0, 115, 0])
        upper_mask = np.array([ 179, 206, 34])

        # Generate mask
        mask = cv2.inRange(hsv, lower_mask, upper_mask)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=5)
        mask = cv2.dilate(mask, kernel, iterations=9)

        # Show mask
        # cv2.imshow("mask", mask)
        # cv2.waitKey(1) & 0xFF

        # Get background mask
        lower_mask_bg = np.array([ 0, 0, 0])
        upper_mask_bg = np.array([ 0, 0, 0])
        mask_bg = cv2.inRange(hsv, lower_mask_bg, upper_mask_bg)
        kernel_bg = np.ones((3, 3), np.uint8)
        mask_bg = cv2.erode(mask_bg, kernel_bg, iterations=5)
        mask_bg = cv2.dilate(mask_bg, kernel_bg, iterations=9)
        self.background = mask_bg
         
        # Get difference between background and object
        diff = cv2.absdiff(self.background,mask)

        # Get contours
        thresh = cv2.threshold(diff,30,255,cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations = 2)
        cnts,res = cv2.findContours(thresh.copy(),
		cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in cnts:
            # Get object rectangle
            (x,y,w,h) = cv2.boundingRect(contour)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0), 3)

        if IMSHOW:
            cv2.imshow("All Contours",frame)
            key = cv2.waitKey(1) 
        
        

        # Get frame size
        height, width, channels = frame.shape
        right = int(width/5*3)
        middle = int(width/5*2)


        if cnts:
            # Save last object
            if self.last_object == None:
                self.last_object = x
            else:
                # Get velocity direction
                self.object_vel = RIGHT if x > self.last_object else LEFT

                # Get object position
                if x > right:
                    self.object_pos = RIGHT
                elif x < middle:
                    self.object_pos = LEFT
                else:
                    self.object_pos = MIDDLE

                # Update last object
                self.last_object = x

        print(self.object_vel, self.object_pos)
        # Check if it's ready to pass through window
        self.check_object()

    def check_object(self):
        # Updates list 
        self.last_vels.pop(0)
        self.last_vels.append(self.object_vel)

        # Get mode of list
        self.object_vel = max(set(self.last_vels), key=self.last_vels.count)

        # Check if it's ready to pass through window
        if self.object_pos == LEFT and self.object_vel == LEFT:
            self.go = True
            print("LESGOOOO")
        elif self.object_pos == RIGHT and self.object_vel == RIGHT:
            self.go = True
            print("LESGOOOO")
        else:
            self.go = False
         



def main():
    rospy.init_node('motion_detection', anonymous=True)
    md = motion_detection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
