import time
import rospy
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class drone():
    def __init__(self):
        rospy.init_node('drone', anonymous=False)
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.goal_pose = PoseStamped()
    def go_to_local(self, goal_x, goal_y, goal_z, yaw = (math.pi/2 * (-1)) , sleep_time=5):
        # rospy.loginfo("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))

        init_time = now = time.time()
        while now-init_time < sleep_time:
            self.set_position(goal_x, goal_y, goal_z, yaw)
            now = time.time()
        
        # rospy.loginfo("Arrived at requested position")
    
    def set_position(self, x, y, z, yaw =(math.pi/2 * (-1))):
        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)
        #self.goal_pose.header.frame_id = ""
        # if yaw == None:
        #     self.goal_pose.pose.orientation = self.drone_pose.pose.orientation
        # else:
        [self.goal_pose.pose.orientation.x, 
        self.goal_pose.pose.orientation.y, 
        self.goal_pose.pose.orientation.z,
        self.goal_pose.pose.orientation.w] = quaternion_from_euler(0,0,yaw) #roll,pitch,yaw
            #print("X: " + str(self.goal_pose)))

        self.setpoint_pub.publish(self.goal_pose)
        

if __name__ == '__main__':
    droone = drone()
    droone.go_to_local(3, 3, 1)
