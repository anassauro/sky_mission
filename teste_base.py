import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time
import numpy as np

#from actuator import Actuator
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Vector3
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSet
from std_msgs.msg import String, Header, Image, Int16MultiArray 
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mav import MAV2


class QRCodeReader():
    def __init__(self) -> None:
        self.qrcode = String()
        self.center_x = 1080
        self.center_y = 720
        self.qrcode_list = []
        self.qrcode_center = Int16MultiArray()
        #rospy.init_node('sky_vision_barcode_analyzer', anonymous=False)
        rospy.Subscriber('/sky_vision/down_cam/qrcode_read', String, self.callback)
        rospy.Subscriber('/sky_vision/down_cam/qrcode_center', Int16MultiArray, self.callback_center)
        rospy.Subscriber('/sky_vision/down_cam/img_raw', Image, self.callback_camera_info)
        self.found_qr = False
        self.found_center = False

    def callback(self, message):
        self.qrcode = message.data
        if self.qrcode not in self.qrcode_list:
            self.qrcode_list.append(self.qrcode)
        self.found_qr = True    

    def callback_center(self, message):
        self.qrcode_center = message.data
        self.found_center = True

    def callback_camera_info(self, message):
        self.center_x = message.width / 2
        self.center_y = message.height / 2

    def centralize(drone, x_center, y_center, offset_x, offset_y):
        x_error = x_center - self.center_x + offset_x
        y_error = y_center - self.center_y + offset_y
        while (abs(x_error) > 5 or abs(y_error) > 5):
            drone.camera_pid(x_error, y_error) 

def main():
    rospy.init_node('mavbase2')
    dr = MAV2()
    qr = QRCodeReader()
    dr.change_auto_speed(1)
    z = 2
    x, y, z = 1, 2, 0       #QR Code distance from takeoff

    try:
        dr.takeoff(z)
        rospy.sleep(5)
        dr.go_to_local([x, y,z+0.5])
        rospy.sleep(5)
        if (not qr.found_qr):
            dr.go_to_local([x, y, z + 0.4])
            rospy.sleep(5)
            dr.go_to_local([x, y, z + 1])
            rospy.sleep(5)
            for i in np.arange(1, 0.4, -0.1):
                dr.go_to_local([x, y, z + i])
                rospy.sleep(2)
        if (not qr.found_qr):
            print ("Not found")
        else:
            print("QR Code found!")
            print("QR Code list:", qr.qrcode_list)
            
            # PID centralize
            if qr.found_center:
                qr.centralize(dr, qr.qrcode_center[0], qr.qrcode_center[1], 0, 40)

            qr.found_qr = False

    except KeyboardInterrupt:
        print("foi")


main()