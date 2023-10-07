import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time

#from actuator import Actuator
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Vector3, Twist
from std_msgs.msg import String, Header, Int16MultiArray
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from tf.transformations import quaternion_from_euler, euler_from_quaternion

TOL = 0.2
TIME_LIMIT = 2

class drone():
    
    def __init__(self) -> None:
        rospy.init_node("teste")
        
        self.centralizing = False
        self.roaming = True
        
        self.goal_pose = PoseStamped()
        self.current_pose = PoseStamped()
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pub_vel = rospy.Publisher('/mavos/setpoint_raw/local', PositionTarget, queue_size = 1)
        self.mode_serv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_callback)
        rospy.Subscriber('/sky_vision/down_cam/pad/bounding_box', Int16MultiArray, self.pad_callback)
        
        print("initiating, goal pose:", self.goal_pose)
        self.pad = list()

    def pad_callback(self, msg):
        point = msg.data
        self.pad = ((point[0] - point[2]/2), (-1*point[1] + point[3]/2))
        print(point)
        print("Pad pos:", self.pad)
        self.last = time.time()
        self.centralizing = True
        
    def centralize(self):
        #constantes para PID
        tolerance = 20
        kp = 0.050
        kd = 0.0050
        ki = 0.00050
        while((time.time() - self.last) < TIME_LIMIT and self.centralizing):
                        
            error = self.pad
            
            if abs(error[0]) < tolerance and abs(error[1]) < tolerance:
                self.mode_serv(0, "LAND")
                self.centralizing = False
                return
                
            last_error = [0, 0] # x e y
            I = [0, 0] # x e y
            vel = PositionTarget()
            vel.velocity.x = error[0] * kp + (error[0] - last_error[0]) * kd + I[0] * ki
            vel.velocity.y = error[1] * kp + (error[1] - last_error[1]) * kd + I[1] * ki
            
            print(vel)
            
            last_error = error
            I[0] = I[0] + error[0]
            I[1] = I[1] + error[1]
            
            self.pub_vel.publish(vel)

        self.centralizing = False
        
    def pos_callback(self, msg):

        self.current_pose.pose.position.x = float(msg.pose.position.x)
        self.current_pose.pose.position.y = float(msg.pose.position.y)
        self.current_pose.pose.position.z = float(msg.pose.position.z)
        
        #print(self.current_pose)

dr = drone()
while(not rospy.is_shutdown()):
    if dr.centralizing:
        dr.centralize()
    
