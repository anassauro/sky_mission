import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time

#from actuator import Actuator
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Vector3
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSet
from std_msgs.msg import String, Header
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mav import MAV2


SIMULATION = True


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


class drone():
    def __init__(self) -> None:
        rospy.init_node("teste")
        
        self.goal_pose = PoseStamped()
        self.current_pose = Vector3(0,0,0)
        self.target_vel = TwistStamped()
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

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

def main():
    rospy.init_node('mavbase2')
    dr = MAV2()
    square_size = 6

    try:
        dr.takeoff(5)
        dr.go_to_local(bases[0], yaw=math.pi/2, sleep_time=2)
        time.sleep(2)
        dr.go_to_local(bases[1], yaw=math.pi/2, sleep_time=2)
        time.sleep(2)
        dr.go_to_local(bases[2], yaw=math.pi/2, sleep_time=2)
    except KeyboardInterrupt:
        print("foi")

    # while not rospy.is_shutdown():
    #     try:
    #         for _ in range(4):
    #             dr.set_position(square_size, 0, 1.2)
    #             time.sleep(5)
    #             dr.set_position(square_size, square_size, 1.2)
    #             time.sleep(5)
    #             dr.set_position(0, square_size, 1.2)
    #             time.sleep(5)
    #             dr.set_position(0, 0, 1.2)
    #             time.sleep(5)

    #             # Diminuir o tamanho do quadrado para a próxima iteração
    #             square_size -= 1
    #             #dr.go_to(-15, 0, 0, 0.25, 0.5)

    #         time.sleep(1)
            # Sair do loop ou adicionar lógica para continuar conforme necessário
        # except KeyboardInterrupt:
        #     print("Cabo")
        #     break

main()
