import rospy

#from actuator import Actuator
from mav import MAV2

def main():
    rospy.init_node('mavbase2')
    dr = MAV2()
    z = 2
    try:
        dr.takeoff(z)
        rospy.sleep(5)
        dr.land()

    except KeyboardInterrupt:
        print("foi")


main()