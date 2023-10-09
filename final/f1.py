from mav import MAV2
import rospy

rospy.init_node('stage_1')

dr = MAV2()

#the place the drone will go so it can start detecting things

search_points = ((1, 1, 2), (-1, -1, 2), (-1, -1, 2), (-1, -1, 2),)



for point in search_points:
    dr.go_to_local((point[0], point[1], point[2]))

    #try to detect thing