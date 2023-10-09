import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time
import cv2

#from actuator import Actuator
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Vector3
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSet
from std_msgs.msg import String, Header
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mav import MAV2


SIMULATION = True
MAX_TIME_ON_BASE = 15


baseA = [0, 2, 2]   #Mais próxima
baseB = [2, 2, 4]
baseC = [8, 8, 1]
baseD = [0, 0, 0]
baseE = [0, 0, 0]
bases = [baseA, baseB, baseC, baseD, baseE]

#leitura da base A dá direção das próximas (ifs)
#lista bases não visitadas
#teste: caso não leia, muda altura
#ajustar descentralização no gotolocal

class QRCodeReader():
    def __init__(self) -> None:
        self.qrcode = String()
        self.qrcode_list = []
        #rospy.init_node('sky_vision_barcode_analyzer', anonymous=False)
        rospy.Subscriber('/sky_vision/down_cam/qrcode_read', String, self.callback)
        self.found_qr = False

    def callback(self, message):
        self.qrcode = message.data
        if self.qrcode not in self.qrcode_list:
            self.qrcode_list.append(self.qrcode)
        self.found_qr = True


"""
class drone():
    def __init__(self) -> None:
        rospy.init_node("teste")
        
        self.goal_pose = PoseStamped()
        self.current_pose = Vector3(0,0,0)
        self.target_vel = TwistStamped()
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.qrcode_detector = QRCodeReader()
        self.found = self.qrcode_detector.found_qr

        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pos_callback)

    def takeoff(self, height):
        rospy.loginfo("TAKING OFF...")
        self.set_mode("GUIDED")
        self.arm()
        rospy.wait_for_service('/mavros/cmd/takeoff', 10)

        rospy.sleep(1)
        response = self.takeoff_srv(altitude=height)
        
        if response.success:
            rospy.loginfo("Takeoff completed!")
            return
        else:
            rospy.loginfo("Takeoff failed!")
            return
        
    def set_position(self, x, y, z, yaw = None):
        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)
        #self.goal_pose.header.frame_id = ""
        # if yaw == None:
        #     self.goal_pose.pose.orientation = self.drone_pose.pose.orientation
        # else:
        #[self.goal_pose.pose.orientation.x, 
        #self.goal_pose.pose.orientation.y, 
        #self.goal_pose.pose.orientation.z,
        #self.goal_pose.pose.orientation.w] = quaternion_from_euler(0,0,yaw) #roll,pitch,yaw
            #print("X: " + str(self.goal_pose)))

        self.setpoint_pub.publish(self.goal_pose)
    
    def go_to_local(self, coordenadas, yaw = None, sleep_time=20):
        # rospy.loginfo("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))

        init_time = now = time.time()
        while not rospy.is_shutdown() and now-init_time < sleep_time:
            self.set_position(coordenadas[0], coordenadas[1], coordenadas[2], yaw)
            now = time.time()
        
        # rospy.loginfo("Arrived at requested position")
    
    def set_position(self, x, y, z, yaw = None):
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

    # def go_to(self, x, y, z, des_velocity, tol):#uses Velocity
    #     if abs(self.current_pose.x - x) > tol:
    #         if x > self.current_pose.x:
    #             self.target_vel.twist.linear.x = des_velocity
    #         else:
    #             self.target_vel.twist.linear.x = -des_velocity
    #         self.vel_pub.publish(self.target_vel) 
    #     else:
    #         self.vel_pub.publish(TwistStamped())
        

    def pos_callback(self, msg):

        
        try:
            self.current_pose.x = float(msg.pose.position.x)
            self.current_pose.y = float(msg.pose.position.y)
            self.current_pose.z = float(msg.pose.position.z)
            print(self.current_pose)
        except:
            pass
    
"""

def main():
    rospy.init_node('mavbase2')
    dr = MAV2()
    qr = QRCodeReader()
    z = 2
    sleep = 5


    try:
        dr.takeoff(z)
        rospy.sleep (7)
        dr.change_auto_speed(1)
        for base in bases:
            dr.go_to_local(base, yaw=math.pi/2, sleep_time=2)

            start = time.time()
            while (not qr.found_qr and time.time() - start < MAX_TIME_ON_BASE):

                try:

                    print(start)

                    print(time.time() - start)
                    
                    print("here")


                except KeyboardInterrupt:
                    print("A")
                    return
                
                finally:
                    time.sleep(1)
            
            qr.found = False

                



        dr.go_to_local([0,0,0], yaw=math.pi/2, sleep_time=2)
        time.sleep(sleep)
        dr.land()

    except KeyboardInterrupt:
        print("cabo")

main()