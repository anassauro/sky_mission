from mav import MAV2
import rospy
from std_msgs.msg import String
import time
import math



HOVER_DISTANCE = 1.8




rospy.init_node('teste_stage_1')

dr = MAV2()
#Bases :                       #em relação ao 0,75
#
baseA = [-2.25, 3, -0.5]   #Mais próxima
baseB = [-3.75, 4.70, -0.5]
baseC = [8, 8, 1]
baseD = [0, 0, 0]
baseE = [0, 0, 0]
basesFixas = [baseA, baseB]
basesMoveis = [baseC, baseD, baseE]
dr.change_auto_speed(1)

def main():
    dr = MAV2()
    sleep = 5

    try:
        for base in basesFixas:
            dr.takeoff(HOVER_DISTANCE)
            rospy.sleep (7)

            dr.go_to_local([base[0], base[1], HOVER_DISTANCE]) #vai p/ XY da base (mantém Z)
            time.sleep(sleep)

            dr.land()
            dr.set_mode('GUIDED')

            time.sleep(sleep)

    except KeyboardInterrupt:
        print ("cabo")

main()
