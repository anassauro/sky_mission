from ..mav import MAV2
import rospy
from std_msgs.msg import String
import time

YOLO_TRIES = 3
WAIT_FOR_REALSENSE = 15
TAKEOFF_DIST = 2
TIME_LIMIT = 2

rospy.init_node('stage_1')

dr = MAV2()

#the place the drone will go so it can start detecting things

search_points = ((1, 1, 2), (-1, -1, 2), (-1, -1, 2), (-1, -1, 2),)

serv = rospy.ServiceProxy('pad_service', String)

dr = MAV2()

for point in search_points:
    dr.go_to_local((point[0], point[1], point[2]))

    rospy.wait_for_service("pad_service")
    for _ in range(YOLO_TRIES):
        request = serv("Chefia naum liberou o aumosso")
        time.sleep(5)
        if(request.data == "Pad Found!"):

            time.sleep(WAIT_FOR_REALSENSE)


            dr.land()

            dr.set_mode('guided')
            time.sleep(5)

            dr.takeoff(TAKEOFF_DIST)
            rospy.sleep(7)

            break
        else:
            print("Pad not found. Trying again...")


dr.go_to_local((0, 0, TAKEOFF_DIST))

dr.land()


