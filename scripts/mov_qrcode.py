import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time
import numpy as np

# from actuator import Actuator
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Vector3
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSet
from std_msgs.msg import String, Header
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sky_mission.sky_utils.communication import Mav


class codeDetector():
    def __init__(self) -> None:
        self.code = String()
        self.code_list = []
        #rospy.init_node('sky_vision_barcode_analyzer', anonymous=False)
        rospy.Subscriber('/sky_vision/code/read', String, self.callback)
        self.found_qr = False

    def callback(self, message):
        self.code = message.data
        if self.code not in self.code_list:
            self.code_list.append(self.code)
        self.found_qr = True    




def main():
    rospy.init_node('mav')
    dr = Mav(debug=True)
    qr = codeDetector()
    # dr.change_auto_speed(1) # We do not have function for it yet
    z = 1,5
    x, y, z = 1, 1, 0     #QR Code distance from takeoff

    try:
        dr.takeoff(z)
        rospy.sleep(5)
        dr.goto(x, y,z)
        rospy.sleep(5)
        if (not qr.found_qr):
            dr.goto(x, y, z - 0.2)
            rospy.sleep(5)
            print ("Not found")
            
        else:
            print("QR Code found!")
            print("QR Code list:", qr.code_list)
            # if qr.code[-1] == 1:
            if qr.code[0] == "E":
                dr.goto(x,y-1,z)
                print("Moving east")
                rospy.sleep(5)
            elif qr.code[0] == "S":
                dr.goto(x-1,y,z)
                print("Moving south")
                rospy.sleep(5)
            elif qr.code[0] == "N":
                dr.goto(x+1,y,z)
                print("Moving north")
                rospy.sleep(5)
            elif qr.code[0] == "W":
                dr.goto(x,y+1,z)
                print("Moving west")
                rospy.sleep(5)
            else:
                print("Sem Movimento")         
        qr.found_qr = False
 
        dr.goto(0,0,z)
        rospy.sleep(5)
        dr.land()
        
    except KeyboardInterrupt:
        print("fim")

main()
