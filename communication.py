#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandHome, ParamSet
from mavros_msgs.srv import CommandTOLRequest, CommandLongRequest, CommandLong, CommandBoolRequest
from mavros_msgs.msg import State, ExtendedState, ParamValue, PositionTarget
import time
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
from dronekit import connect, VehicleMode

class Communication():

    def __init__(self, DEBUG):
        ############ private Attributes #################
        try:
            self.__drone = connect('localhost:14550', wait_ready=True)
        except:
            rospy.logerr("Could not connect with dronekit")
        
        rospy.loginfo("Drone Connected")
        self.__wait_time = 2
        self.DEBUG = DEBUG
        self.goal_pose = PoseStamped()
        self.drone_pose = PoseStamped() 
        
        ############# Services ##################
        self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.set_home_srv = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
        rospy.loginfo("Services are up")
        
        ############### Publishers ##############
        self.local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 20)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',  TwistStamped, queue_size=5)
        self.target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=5)
        rospy.loginfo("Publishers are up")

        ########## Subscribers ##################
        self.local_atual = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_callback)
        #self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback, queue_size=10)
        #self.extended_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extended_state_callback, queue_size=2) 
        rospy.loginfo("Subscribers are up")

    ############ setters #################
    def set_mode (self, mode: str) -> bool:
        self.__drone.mode = VehicleMode(mode)
        rospy.sleep(self.__wait_time)
        if self.__drone.mode.name == mode:
            if self.DEBUG: rospy.loginfo(f"Changed mode to {mode}")
            return True
        else:
            if self.DEBUG: rospy.logerr(f"ERROR to change mode, It is current {self.__drone.mode.name}")
            return False

    
    
    ############ getters #################
    def get_state (self):
        return self.__drone.nome.name


    ########## Callback functions ###########
    def local_callback(self, data):
        self.drone_pose = data

    ############ behaviral functions #################
    def set_home(self) -> bool:
        request = CommandHome()
        request.current_gps = False
        request.yaw = 0
        request.latitude = 0 
        request.longitude = 0
        request.altitude = 0
        rospy.wait_for_service('/mavros/set_home', 15)
        response = self.set_home_srv(request)
        
        if response.success:
            rospy.loginfo("Home set!")
        else:
            rospy.loginfo("Home not set!")
        
        return response.success
    

    def arm (self) -> bool:
        if self.__drone.mode.name != "GUIDED":
            if self.DEBUG: rospy.logerr(f"Drone is not in GUIDED")
            return False
        
        self.__drone.armed = True
        rospy.sleep(self.__wait_time)
        if self.__drone.armed:
            if self.DEBUG: rospy.loginfo(f"Drone has been armed")
            return True
        else:
            if self.DEBUG: rospy.logerr(f"ERROR to arm drone")
            return False

    def takeoff (self, altitude: int = 1) -> bool:
        if not self.__drone.armed:
            if self.DEBUG: rospy.logerr(f"Drone is not armed, cannot takeoff")
            return False
        
        if self.DEBUG: rospy.loginfo(f"Drone is taking off {altitude} meter(s)")
        self.__drone.simple_takeoff(altitude)
    
    def land (self) -> bool:
        self.__drone.mode = VehicleMode("LAND")

        rospy.sleep(self.__wait_time)
        if self.__drone.mode.name == "LAND":
            if self.DEBUG: rospy.loginfo(f"Drone is landing")
            return True
        else:
            if self.DEBUG: rospy.logerr(f"ERROR to land drone")
            return False

    ####### Goal Position and Velocity #########

    def set_position (self, x, y, z, yaw = None):
        self.goal_pose.pose.position.x = float(x)
        self.goal_pose.pose.position.y = float(y)
        self.goal_pose.pose.position.z = float(z)
        self.goal_pose.header.frame_id = self.drone_pose.header.frame_id
        if yaw == None:
            self.goal_pose.pose.orientation = self.drone_pose.pose.orientation
        else:
            [self.goal_pose.pose.orientation.x, 
            self.goal_pose.pose.orientation.y, 
            self.goal_pose.pose.orientation.z,
            self.goal_pose.pose.orientation.w] = quaternion_from_euler(0,0,yaw) #roll,pitch,yaw

        self.local_position_pub.publish(self.goal_pose)

    def set_position_target(self, x_position=0, y_position=0, z_position=0, x_velocity=0, y_velocity=0, z_velocity=0, x_aceleration=0, y_aceleration=0, z_aceleration=0, yaw=0, yaw_rate=0, coordinate_frame = PositionTarget.FRAME_LOCAL_NED, type_mask=0b1111110111111000):
        self.pose_target.coordinate_frame = coordinate_frame #Use PositionTarget.FRAME_LOCAL_NED para movimento relativo ao corpo do drone  
        self.pose_target.type_mask = type_mask
        #https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK

        self.pose_target.position.x = x_position
        self.pose_target.position.y = y_position
        self.pose_target.position.z = z_position

        self.pose_target.velocity.x = x_velocity
        self.pose_target.velocity.y = y_velocity
        self.pose_target.velocity.z = z_velocity

        self.pose_target.acceleration_or_force.x = x_aceleration
        self.pose_target.acceleration_or_force.y = y_aceleration
        self.pose_target.acceleration_or_force.z = z_aceleration

        self.pose_target.yaw = yaw
        self.pose_target.yaw_rate = yaw_rate

        self.target_pub.publish(self.pose_target)


    
    def go_to_local(self, coordenadas, yaw = None, sleep_time=5):
        #rospy.loginfo("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))
        init_time = now = time.time()
        while not rospy.is_shutdown() and now-init_time < sleep_time:
            self.set_position(coordenadas[0], coordenadas[1], coordenadas[2], yaw)
            now = time.time()
        
        rospy.loginfo("Arrived at requested position")

    
    def change_auto_speed(self, guided_vel): # Velocity of guided mode in m/s
        rospy.loginfo("Changing speed to " + str(guided_vel))
        self.set_param('WPNAV_SPEED', float(guided_vel*100))


    def set_vel(self, x, y, z, yaw = 0):
        self.goal_vel.twist.linear.x = float(x)
        self.goal_vel.twist.linear.y = float(y)
        self.goal_vel.twist.linear.z = float(z)

        self.goal_vel.twist.angular.z = float(yaw)
        self.velocity_pub.publish(self.goal_vel)    



####### testes #########

if __name__ == "__main__":
    #rospy.loginfo("start code")
    square_half_size = 0.5
    try:
        rospy.init_node('Test_Comunication')
        drone = Communication(DEBUG = True)
        #drone.set_home()
        drone.set_mode("GUIDED")
        #drone.arm()
        rospy.sleep(5)
        #drone.takeoff()
        #rospy.sleep(5)
        drone.go_to_local(coordenadas = [0,0,2*square_half_size])
        rospy.sleep(5)
        #drone.go_to_local(coordenadas = [2*square_half_size,0,2*square_half_size])
        #rospy.sleep(5)
        #drone.go_to_local(coordenadas = [square_half_size,square_half_size,2*square_half_size])
        #rospy.sleep(5)
        drone.go_to_local(coordenadas = [0,2*square_half_size,2*square_half_size])
        rospy.sleep(5)
        #drone.go_to_local(coordenadas = [0,0,2*square_half_size])
        #rospy.sleep(5)
        drone.land()
        rospy.sleep(5)
    except Exception as e:
        rospy.logerr("Deu ruim")
        rospy.logerr(str(e))
