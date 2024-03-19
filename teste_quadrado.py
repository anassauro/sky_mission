import rospy

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TwistStamped

class Main():

    def __init__(self):
        rospy.init_node('teste_quadrado', anonymous=True)
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.rate = rospy.Rate(60)
    
    def mission(self):
        rospy.loginfo('Mission started')
        rospy.sleep(5)
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 1
        pose.pose.position.z = 1
        [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_euler(0, 0, 0)
        self.local_position_pub.publish(pose)
        rospy.sleep(5)
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 1
        [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_euler(0, 0, 0)
        self.local_position_pub.publish(pose)
    
    def land(self):
        rospy.loginfo('Landing')
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_euler(0, 0, 0)
        self.local_position_pub.publish(pose)
        rospy.sleep(5)
        rospy.loginfo('Landed')


        


if __name__ == '__main__':
    main = Main()
    main.mission()
    main.land()
    rospy.spin()