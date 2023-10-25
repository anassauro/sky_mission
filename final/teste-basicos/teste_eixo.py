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
    try:
        dr.takeoff(z)
        rospy.sleep(7)
        dr.go_to_local([1, 0, z], yaw=math.pi/2, sleep_time=5)  #eixo X
        time.sleep(sleep)
        dr.go_to_local([0, 1, z], yaw=math.pi/2, sleep_time=2)  #eixo Y
        time.sleep(sleep)
        dr.go_to_local([0, 0, z+1], yaw=math.pi/2, sleep_time=2)  #eixo Z
        time.sleep(sleep)
        dr.go_to_local([0, 0, 0], yaw=math.pi/2, sleep_time=2)
        time.sleep(sleep)
        dr.land()
    except KeyboardInterrupt:
        print("foi")


main()