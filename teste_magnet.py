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

def main():
    rospy.init_node('mavbase2')
    dr = MAV2()
    z = 2
    x, y, z = 0,0,1

    electromagnet = 11
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(electromagnet, GPIO.OUT)

    while True:
        try:
            dr.takeoff(z)
            rospy.sleep(7)
            dr.go_to_local([x+1,y+1,z])
            rospy.sleep(7)
            if (x, y, z != [0,0,1]):
                GPIO.output(electromagnet, True)
                for i in np.arrange(1,0.5, -0,1):
                    dr.go_to_local(x,y,z + i)
                    rospy.sleep(3)
                dr.go_to_local(x-1,y-1,z+0,5)
                rospy.sleep(5)
            else:
                GPIO.output(electromagnet, False)
                
        except KeyboardInterrupt:
            print("fim")       

main()



