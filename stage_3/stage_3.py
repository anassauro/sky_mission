import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time

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
    dr = drone()
    time.sleep(1)
    step = 1
    x,y,z = 1, 5, 1
    x_start, z_start = x, z
    
    while not rospy.is_shutdown():
        
        try:
            
            if step == 1:
                if(dr.go_to(x, y, z)):
                    step += 1
                    
            elif step == 2:
                if(dr.go_to(x, y , z)):
                    x += 0.5
                    time.sleep(1)
                    
                    if(x > 5):
                        x = x_start
                        step += 1
                        
            elif step == 3:
                if(dr.go_to(x, y , z)):
                    z += 0.5
                    
                    if(z > 3):
                        step += 1
                    else:
                        step = 2

            else:
                if(dr.go_to(0, 0 , 3)):
                    print("END OF MISSION")
                
            

                
            """
            print("looping")
                    
            point = [2, 6, 1]
            
            dr.go_to(point[0], point[1], point[2])
            
            print("second")
            
            for z in range(4):  # Realizar os movimentos de ida e volta 4 vezes
                
                dr.go_to(point[0], point[1], point[2] + z)
                for x in range(5):
                    
                    dr.go_to(point[0] + x, point[1], point [2] + z)

            dr.go_to(0, 0, 1)
            """
            
        except KeyboardInterrupt:
            print("Process killed by user.")
        
        

main()