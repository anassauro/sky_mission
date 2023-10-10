from mav import MAV2
import time
import rospy

dr = MAV2()

bases = [(-4.95, 0.9, -0.5), (-4.05, 5.3, -0.5), (-1.85, 3.6, -0.5), (-0.25, 7, 1), (-2.75, 0, 1.5)]

HOVER_DISTANCE = 2.5
WAIT_FOR_DETECTION = 30
NUM_SHAKE = 4

try:
    dr.set_mode('GUIDED')

    dr.takeoff(HOVER_DISTANCE)
    rospy.time(7)
    
    for base in bases:
        dr.go_to_local(base[0], base[1], HOVER_DISTANCE)
        for _ in range(NUM_SHAKE):

            time.sleep(WAIT_FOR_DETECTION/NUM_SHAKE)
            print("shaking...")
    
except KeyboardInterrupt:
    dr.go_to_local(0, 0, HOVER_DISTANCE)
    dr.land()

dr.go_to_local(0, 0, HOVER_DISTANCE)
dr.land()

