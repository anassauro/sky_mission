from mav import MAV2
import rospy
rospy.init_node('teste')

dr = MAV2()

dr.takeoff(1)

HOVER_DISTANCE = 2
WAIT_FOR_DETECTION = 30
NUM_SHAKE = 4

pos = (1, 1, 1)

dr.go_to_local(pos)
