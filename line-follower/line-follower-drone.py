#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import time
import math

# Ros messages
from geometry_msgs.msg import Vector3, Point
from mavros_msgs.msg import PositionTarget
from tf2_msgs.msg import TFMessage

# Activate imshow
IMSHOW = True
# Activate video recording
RECORD = False
TESTNUM = 1

# Activate only yaw control
ONLY_YAW = False



class line_follower:

    def __init__(self):        
        # Send linear and angular velocity
        self.setpoint_pub_ = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        # Get position and orientation
        rospy.Subscriber('/tf', TFMessage, self.dronePosCallback)

        # Get camera image in simulation
        # self.image_sub = rospy.Subscriber('/webcam/image_raw', Image, self.callback)

        # Velocity
        self.velocity = 0.3
        # min area of the line
        self.minArea = 5000

        # PID parameters
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
        
        # Get camera image
        self.capture = cv2.VideoCapture("./video-line3.avi")
        # Stores the position of the drone
        self.drone_pos_ = Point()

        ## Record
        if RECORD:
            self.testnum = TESTNUM
            self.frame_width = int(self.capture.get(3))
            self.frame_height = int(self.capture.get(4))
            self.size = (self.frame_width, self.frame_height)
            self.result = cv2.VideoWriter(f'video-liha-{self.testnum}.avi', 
                            cv2.VideoWriter_fourcc(*'MJPG'),
                            10, self.size)



    def dronePosCallback(self, data):
        try:
            # Get drone position
            self.dronePos = data.transforms.transform.translation
            self.drone_pos_.x = self.dronePos.x
            self.drone_pos_.y = self.dronePos.y
            self.drone_pos_.z = self.dronePos.z
        except:
            pass   
        return


    def line_detect(self, cv_image):

        ## Generate mask
        #  Blue mask
        lower_mask = np.array([ 69, 69, 37])
        upper_mask = np.array(  [ 157, 255, 255])
        mask = cv2.inRange(cv_image, lower_mask, upper_mask)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=5)
        mask = cv2.dilate(mask, kernel, iterations=9)

        # Show mask 
        if IMSHOW:
            cv2.imshow("mask", mask)
            cv2.waitKey(1) & 0xFF

        # Find contours in mask
        contours_blk, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk = list(contours_blk)

        # If there is a line
        if len(contours_blk) > 0:

            # starts looking from left to right
            contours_blk.sort(key=cv2.minAreaRect)

            # if the area of the contour is greater than minArea
            if cv2.contourArea(contours_blk[0]) > self.minArea:
                
                blackbox = cv2.minAreaRect(contours_blk[0])
                (x_min, y_min), (w_min, h_min), angle = blackbox

                # fix angles
                if angle < -45:
                    angle = 90 + angle
                if w_min < h_min and angle > 0:
                    angle = (90 - angle) * -1
                if w_min > h_min and angle < 0:
                    angle = 90 + angle

                # Rotate image
                angle += 90

                # Optmize angle 
                if angle > 90:
                    angle = angle - 180

                # Get distance from center of image
                setpoint = cv_image.shape[1] / 2
                error = int(x_min - setpoint)

                ## Error correction (y axis)
                normal_error = float(error) / setpoint

                if error > 0:
                    self.line_side = 1  # line in right
                elif error <= 0:
                    self.line_side = -1  # line in left

                self.integral = float(self.integral + normal_error)
                self.derivative = normal_error - self.last_error
                self.last_error = normal_error
                error_corr = -1 * (self.Kp * normal_error + self.Ki * self.integral + self.kd * self.derivative)  # PID controler

                ## Angle correction (z axis)
                angle = int(angle)

                self.integral_ang = float(self.integral_ang + angle)
                self.derivative_ang = angle - self.last_ang
                self.last_ang = angle
                ang_corr = -1 * (self.Kp_ang * angle + self.Ki_ang * self.integral_ang + self.kd_ang * self.derivative_ang)  # PID controler

                ## Set velocity
                setpoint_ = PositionTarget()
                vel_setpoint_ = Vector3()

                vel_setpoint_.x = self.velocity
                vel_setpoint_.y = error_corr
                yaw_setpoint_ = ang_corr * math.pi / 180 * 5

                # Limit y velocity
                if vel_setpoint_.y > self.velocity:
                    vel_setpoint_.y = self.velocity

                print(f"x: {vel_setpoint_.x} | y: {vel_setpoint_.y} | yaw: {yaw_setpoint_}")

                setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
                
                if  ONLY_YAW:
                    setpoint_.velocity.x = 0
                    setpoint_.velocity.y = 0
                else:
                    setpoint_.velocity.x = vel_setpoint_.x
                    setpoint_.velocity.y = vel_setpoint_.y

                setpoint_.position.z = self.drone_pos_.z
                setpoint_.yaw = yaw_setpoint_

                self.setpoint_pub_.publish(setpoint_)



    # Zoom-in the image
    def zoom(self, cv_image, scale):
        if cv_image is None: return None
        height, width, _ = cv_image.shape

        # Crop the image
        centerX, centerY = int(height / 2), int(width / 2)
        radiusX, radiusY = int(scale * height / 100), int(scale * width / 100)

        minX, maxX = centerX - radiusX, centerX + radiusX
        minY, maxY = centerY - radiusY, centerY + radiusY

        cv_image = cv_image[minX:maxX, minY:maxY]
        cv_image = cv2.resize(cv_image, (width, height))

        return cv_image


    def detection_loop(self):
        while self.capture.isOpened():
            
            ret, cv_image = self.capture.read()
            if ret:
                # Record video if activated
                if RECORD:
                    self.result.write(cv_image)

                # Zoom in    
                cv_image = self.zoom(cv_image, scale=20)
                cv_image = cv2.add(cv_image, np.array([-50.0]))
                if IMSHOW:
                    cv2.imshow("Image", cv_image)
                    cv2.waitKey(1) & 0xFF

                # Convert to HSV
                cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

                # Detect line
                self.line_detect(cv_image_hsv)

            else:
                break
        


def main():
    rospy.init_node('line_follower', anonymous=True)
    lf = line_follower()
    time.sleep(3)
    try:
        lf.detection_loop()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
