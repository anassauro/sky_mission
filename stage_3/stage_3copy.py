import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time
from mav import MAV2

#from actuator import Actuator
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Vector3
from std_msgs.msg import String, Header
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler, euler_from_quaternion

TOL = 0.2

class BarcodeAnalyzer():
    
    def __init__(self) -> None:
        self.barcode = String()
        self.barcode_list = []
        #rospy.init_node('sky_vision_barcode_analyzer', anonymous=False)
        
        rospy.Subscriber('/sky_vision/down_cam/barcode_read', String, self.callback)
    
    def callback(self, message):
        self.barcode = message.data
        if self.barcode not in self.barcode_list:
            self.barcode_list.append(self.barcode)

class drone():
    
    def __init__(self) -> None:
        rospy.init_node("teste")
        
        self.bc = BarcodeAnalyzer()
        self.goal_pose = PoseStamped()
        self.current_pose = PoseStamped()
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pos_callback)
        print("initiating, goal pose:", self.goal_pose)
        
    def set_position(self, x, y, z, yaw = None):
        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)

        self.setpoint_pub.publish(self.goal_pose)

    def go_to(self, x, y, z, yaw = None):
        
        print("Going to:", x, y, z, yaw)
        print("Currently at: ", self.current_pose)
        
        if(abs(x - self.current_pose.pose.position.x) < TOL and
              abs(y - self.current_pose.pose.position.y) < TOL and
              abs(z - self.current_pose.pose.position.z) < TOL):
            return True
        
        self.set_position(x, y, z, yaw)
        print(self.current_pose)
        time.sleep(0.2)
        return False
        

    def pos_callback(self, msg):

        self.current_pose.pose.position.x = float(msg.pose.position.x)
        self.current_pose.pose.position.y = float(msg.pose.position.y)
        self.current_pose.pose.position.z = float(msg.pose.position.z)
        
        #print(self.current_pose)

def main():
    rospy.init_node('mavbase2')
    dr = MAV2()
    time.sleep(1)
    step = 1
    x = 1
    y = 5
    z = 1
    local = [x, y, z]
    #x_start, z_start = x, z
    sleep = 5
    z = 1
    
    while not rospy.is_shutdown():
        
        try:
            dr.takeoff(2)
            rospy.sleep(7)
            dr.go_to_local(local, yaw=math.pi/2, sleep_time=2)
            while (z < 4):
                while (x < 5):
                    x += 0.5
                    local = [x, y, z]
                    dr.go_to_local(local, yaw=math.pi/2, sleep_time=3)

                z += 0.5
                local = [x, y, z]

                while (x > 1):
                        x -= 0.5
                        local = [x, y, z]
                        dr.go_to_local(local, yaw=math.pi/2, sleep_time=3)
                
                z += 0.5
                local = [x, y, z]
            local = [0,0,0]
            dr.go_to_local(local, yaw=math.pi/2, sleep_time=3)
            print ("End of mission")
                
            
        except KeyboardInterrupt:
            print("Process killed by user.")
        
        

main()