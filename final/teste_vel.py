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

def teste_vel_guided():
    rospy.init_node('mavbase2')
    dr = MAV2()
    z = 1
    sleep = 5

    dr.change_auto_speed(1)  #m/s
    dr.go_to_local([1, 1, z], yaw=math.pi/2, sleep_time=2)
    time.sleep(sleep)
    dr.change_auto_speed(3)
    dr.go_to_local([0, 0, z], yaw=math.pi/2, sleep_time=2)
    time.sleep(sleep)


def teste_vel_set():
    rospy.init_node('mavbase2')
    dr = MAV2()
    z = 2
    sleep = 5
    dr.set_vel(1, 0, 0)
    time.sleep(2)
    dr.set_vel(0, 0, 0)
    time.sleep(5)

teste_vel_guided()
time.sleep(5)
#teste_vel_set()

