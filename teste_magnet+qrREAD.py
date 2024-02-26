import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time
import numpy as np
import RPi.GPIO as GPIO

#from actuator import Actuator
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Vector3
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSet
from std_msgs.msg import String, Header
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mav import MAV2


class QRCodeReader():
    def __init__(self) -> None:
        self.qrcode = String()
        self.qrcode_list = []
        #rospy.init_node('sky_vision_barcode_analyzer', anonymous=False)
        rospy.Subscriber('/sky_vision/down_cam/qrcode_read', String, self.callback)
        self.found_qr = False

    def callback(self, message):
        self.qrcode = message.data
        if self.qrcode not in self.qrcode_list:
            self.qrcode_list.append(self.qrcode)
        self.found_qr = True    




def main():
    rospy.init_node('mavbase2')
    dr = MAV2()
    qr = QRCodeReader()
    dr.change_auto_speed(1)
    z = 1
    x, y = 1, 2       #QR Code distance from takeoff

    electromagnet = 6
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(electromagnet, GPIO.OUT)

    while not qr.found_qr:
        try:
            dr.takeoff(z)
            rospy.sleep(5)
            dr.go_to_local([x, y, z + 0.5])
            rospy.sleep(5)
            
            while z > 0.2 and not qr.found_qr:
                dr.go_to_local([x, y, z])
                rospy.sleep(2)
                z -= 0.1
            
            if not qr.found_qr:
                GPIO.output(electromagnet, False)
                dr.go_to_local([x, y, z + 0.4])
                rospy.sleep(5)
                
                for i in np.arange(1, 0.4, -0.1):
                    dr.go_to_local([x, y, z + i])
                    rospy.sleep(2)
                    
                if not qr.found_qr:
                    print("Not found")
                    rospy.sleep(2)
                    dr.go_to_local([x, y, 1.5])
                    rospy.sleep(2)
                    dr.go_to_local([0, 0, 1.5])
                    rospy.sleep(2)
                    dr.land()

                else:
                    print("QR Code found!")
                    print("QR Code list:", qr.qrcode_list)
                    GPIO.output(electromagnet, True)
                    dr.go_to_local([x, y, 0])
                    qr.found_qr = False
                    rospy.sleep(2)
                    dr.go_to_local([x, y, 1.5])
                    rospy.sleep(2)
                    dr.go_to_local([0, 0, 1.5])
                    rospy.sleep(2)
                    dr.land()
            else:
                    print("QR Code found!")
                    print("QR Code list:", qr.qrcode_list)
                    GPIO.output(electromagnet, True)
                    dr.go_to_local([x, y, 0])
                    qr.found_qr = False
                    rospy.sleep(2)
                    dr.go_to_local([x, y, 1.5])
                    rospy.sleep(2)
                    dr.go_to_local([0, 0, 1.5])
                    rospy.sleep(2)
                    dr.land()

        except KeyboardInterrupt:
            print("foi")



main()
