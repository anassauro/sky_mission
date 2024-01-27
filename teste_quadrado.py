import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time

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
    sleep = 5
    square_size = 1.5
    tol_time = 15
    try:
        dr.takeoff(z)
        rospy.sleep(5)
        dr.go_to_local([-square_size, 0, z])
        time.sleep(5)
        dr.go_to_local([-square_size, square_size, z])
        time.sleep(5)
        dr.go_to_local([0, square_size, z])
        time.sleep(5) 
        dr.go_to_local([0, 0, z])
        time.sleep(5)
        dr.go_to_local([0, 0, z+1])
        time.sleep(5)
        dr.land()

    except KeyboardInterrupt:
        print("foi")


main()