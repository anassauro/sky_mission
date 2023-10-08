import rospy
import time
import math
import dronekit
from pymavlink import mavutil
import time
from mav import MAV2

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
        
        mav = MAV2()
        self.step = "ROAMING" # TAKEOFF, ROAMING, CENTRALIZING, LANDING 
        self.roam_step = "CENTER" # CENTER, NORTH, EAST, SOUTH, WEST, NORTH_EAST, SOUTH_EAST, SOUTH_WEST, NORTH_WEST
        self.dir_list = ["NORTH", "EAST", "SOUTH", "WEST", "NORTH_EAST", "SOUTH_EAST", "SOUTH_WEST", "NORTH_WEST"]
        self.arena_size = 10 # tamanho do lado da arena em metros
        self.roam_vel = 0.5 # velocidade de roaming em m/s
        rospy.Subscriber('/sky_vision/down_cam/pad/bounding_box', Int16MultiArray, self.pad_callback)
        
        
        self.pad = list()
        self.pad_coords = []

    def pad_callback(self, msg):
        point = msg.data
        self.pad = ((point[0] - point[2]/2), (-1*point[1] + point[3]/2))
        print(point)
        print("Pad pos:", self.pad)
        self.last = time.time()
        for i in self.pad_coords:
            if abs(i.position.posex - self.mav.drone_pose.position.pose.x) > 1 and (i.position.pose.y - self.mav.drone_pose.position.pose.y) > 1:
                self.step = "CENTRALIZING"
        
    def centralize(self):
        #constantes para PID
        tolerance = 20
        kp = 0.050
        kd = 0.0050
        ki = 0.00050
        if((time.time() - self.last) < TIME_LIMIT):
                        
            error = self.pad
            
            if abs(error[0]) < tolerance and abs(error[1]) < tolerance:
                self.mode_serv(0, "LAND")
                self.step = "LANDING"
                return
                
            last_error = [0, 0] # x e y
            I = [0, 0] # x e y
            
            vel_x = error[0] * kp + (error[0] - last_error[0]) * kd + I[0] * ki
            vel_y = error[1] * kp + (error[1] - last_error[1]) * kd + I[1] * ki
            
            
            last_error = error
            I[0] = I[0] + error[0]
            I[1] = I[1] + error[1]
            
            self.mav.set_vel(vel_x, vel_y, 0)

        self.step = "TAKEOFF"
        
        
        #print(self.current_pose)
    def roaming(self):
        #Flies around the arena, looking for pads
        if self.roam_step == "CENTER":
            x_dir = (-self.mav.drone_pose.position.pose.x + self.arena_size/2)/abs(-self.mav.drone_pose.position.pose.x + self.arena_size/2)
            y_dir = (-self.mav.drone_pose.position.pose.y + self.arena_size/2)/abs(-self.mav.drone_pose.position.pose.y + self.arena_size/2)
            
            y_vel = y_dir * self.roam_vel
            if abs(self.mav.drone_pose.position.pose.x - self.arena_size/2) > TOL:
                x_vel = x_dir * self.roam_vel
            else:
                x_vel = 0
            if abs(self.mav.drone_pose.position.pose.y - self.arena_size/2) > TOL:
                y_vel = y_dir * self.roam_vel
            else:
                y_vel = 0
            if x_vel == 0 and y_vel == 0:
                if len(self.dir_list) == 0:
                    self.dir_list = ["NORTH", "EAST", "SOUTH", "WEST", "NORTH_EAST", "SOUTH_EAST", "SOUTH_WEST", "NORTH_WEST"]
                self.roam_step = self.dir_list.pop()
                self.mav.set_vel(0,0,0)
                print("CENTERED")
            self.mav.set_vel(x_vel, y_vel, 0)
        elif self.roam_step == "NORTH":
            if abs(self.mav.drone_pose.position.pose.x - self.arena_size) > TOL:
                self.mav.set_vel(self.roam_vel,0,0)
            else:
                self.roam_step = "CENTER"
                self.mav.set_vel(0,0,0)
                print("NORTH")
        elif self.roam_step == "EAST":
            if abs(self.mav.drone_pose.position.pose.y - self.arena_size) > TOL :
                self.mav.set_vel(0,self.roam_vel,0)
            else:
                self.roam_step = "CENTER"
                self.mav.set_vel(0,0,0)
                print("EAST")
        elif self.roam_step == "SOUTH":
            if abs(self.mav.drone_pose.position.pose.x) > TOL:
                self.mav.set_vel(-self.roam_vel,0,0)
            else:
                self.roam_step = "CENTER"
                self.mav.set_vel(0,0,0)
                print("SOUTH")
        elif self.roam_step == "WEST":
            if abs(self.mav.drone_pose.position.pose.y) > TOL:
                self.mav.set_vel(0,-self.roam_vel,0)
            else:
                self.roam_step = "CENTER"
                self.mav.set_vel(0,0,0)
                print("WEST")
        elif self.roam_step == "NORTH_EAST":
            if abs(self.mav.drone_pose.position.pose.x - self.arena_size) > TOL and abs(self.mav.drone_pose.position.pose.y - self.arena_size) > TOL:
                self.mav.set_vel(self.roam_vel,self.roam_vel,0)
            else:
                self.roam_step = "CENTER"
                self.mav.set_vel(0,0,0)
                print("NORTH_EAST")
        elif self.roam_step == "SOUTH_EAST":
            if abs(self.mav.drone_pose.position.pose.x) > TOL and abs(self.mav.drone_pose.position.pose.y - self.arena_size) > TOL:
                self.mav.set_vel(-self.roam_vel,self.roam_vel,0)
            else:
                self.roam_step = "CENTER"
                self.mav.set_vel(0,0,0)
                print("SOUTH_EAST")
        elif self.roam_step == "SOUTH_WEST":
            if abs(self.mav.drone_pose.position.pose.x) > TOL and abs(self.mav.drone_pose.position.pose.y) > TOL:
                self.mav.set_vel(-self.roam_vel,-self.roam_vel,0)
            else:
                self.roam_step = "CENTER"
                self.mav.set_vel(0,0,0)
                print("SOUTH_WEST")
        elif self.roam_step == "NORTH_WEST":
            if abs(self.mav.drone_pose.position.pose.x - self.arena_size) > TOL and abs(self.mav.drone_pose.position.pose.y) > TOL:
                self.mav.set_vel(self.roam_vel,-self.roam_vel,0)
            else:
                self.roam_step = "CENTER"
                self.mav.set_vel(0,0,0)
                print("NORTH_WEST")

    def takeoff(self):
        #takeoff
        self.mav.takeoff(2)
        if self.mav.drone_pose.position.pose.z > 1.5:
            self.step = "ROAMING"
            self.mav.set_vel(0,0,0)
            print("TAKEOFF")
    
    def land(self):
        #land
        self.mav.land()
        if self.mav.drone_pose.position.pose.z < 0.2:
            self.pad_coords.append(self.mav.drone_pose.position.pose)
            self.mav.set_vel(0,0,0)
            print("LANDING")
            time.sleep(1)
            self.step = "TAKEOFF"



if __name__ == "__main__":
    dr = drone()
    time.sleep(5)

    while(not rospy.is_shutdown()):
        try:
            if dr.step == "TAKEOFF":
                dr.takeoff()
            elif dr.step == "ROAMING":
                dr.roaming()
            elif dr.step == "CENTRALIZING":
                dr.centralize()
            elif dr.step == "LANDING":
                dr.land()
        except KeyboardInterrupt:
            print("Shutting down")
            break

    
