import rospy
import time
import numpy as np
import RPi.GPIO as GPIO
from std_msgs.msg import String

from mav import MAV2

def main():
    rospy.init_node('mavbase2')
    dr = MAV2()
    x, y, z = 0,0,1

    electromagnet = 11
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(electromagnet, GPIO.OUT)
    GPIO.output(electromagnet, True)

    while True:
        try:
            dr.takeoff(z)
            rospy.sleep(7)
            dr.go_to_local([x+1,y+1,z])
            rospy.sleep(7)
            GPIO.output(electromagnet, False)
            print("soltei a carga")
            rospy.sleep(5)
            dr.go_to_local([0, 0, 1])
            dr.land()

                
        except KeyboardInterrupt:
            print("fim")       

main()



