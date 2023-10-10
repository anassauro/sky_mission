from mav import MAV2
import time
import rospy

dr = MAV2()

base = (0.6, 0.6, 0)

HOVER_DISTANCE = 1
WAIT_FOR_DETECTION = 30
NUM_SHAKE = 4

try:
    dr.set_mode('GUIDED')

    dr.takeoff(1)
    rospy.time(7)

    dr.go_to_local(base[0], base[1], base[2] + HOVER_DISTANCE)
    for _ in range(NUM_SHAKE):

        time.sleep(WAIT_FOR_DETECTION/NUM_SHAKE)
        print("shaking...")
    
except KeyboardInterrupt:
    dr.go_to_local(0, 0, HOVER_DISTANCE)
    dr.land()

