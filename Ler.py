#!/usr/bin/python3

# Revives the realsense camera if it dies!
# Name 'Ler.py' is shit but it is legacy, sorry about that 

import rospy
import time
import subprocess
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

SLEEP_THRESHOLD = 0.01 # s

class  realsensor():
  def __init__(self):
    self.cur_time = -1
    self.thresh = 1
    self.cont = 0
    self.realaunch = subprocess.Popen(["roslaunch", "realsense2_camera", "rs_t265.launch", "--wait"])
    rospy.init_node('gambiarra', anonymous = True)
    rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, self.pose_callback)
    rospy.Subscriber("gambiarra/setpoint_local", PoseStamped, self.setpoint_callback)
    self.pose_pub = rospy.Publisher("gambiarra/local_pose", PoseStamped, queue_size = 1)
    self.setpoint_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)
    self.oldCoord = PoseStamped()
    self.cur_pos = PoseStamped()
    self.hasReset = False
  
  def coord_fix(self, newCoord):
    
    newCoord.pose.position.x += self.oldCoord.pose.position.x
    newCoord.pose.position.y += self.oldCoord.pose.position.y
    newCoord.pose.position.z += self.oldCoord.pose.position.z
    return newCoord
  
  def coord_break(self, newCoord):
    newCoord.pose.position.x -= self.oldCoord.pose.position.x
    newCoord.pose.position.y -= self.oldCoord.pose.position.y
    newCoord.pose.position.z -= self.oldCoord.pose.position.z
    return newCoord
  
  def setpoint_callback(self, data):
    if self.hasReset:
      self.setpoint_pub.publish(self.coord_break(data))
    else:
      self.setpoint_pub.publish(data)
  
  def pose_callback(self, data):
    self.cur_time = time.time()
    if self.hasReset:
      self.cur_pos = self.coord_fix(data)
    else:
      self.cur_pos = data
    print(self.cur_pos)
  
  def healthChecker(self):
    if self.cur_time != -1 and (time.time() - self.cur_time) > self.thresh:
      self.oldCoord = self.cur_pos
      self.hasReset = True
      self.realaunch.terminate()
      time.sleep(1)
      print("restarting realsense..")
      self.realaunch = subprocess.Popen(["roslaunch", "realsense2_camera", "rs_t265.launch"])
      self.cur_time = -1

if __name__ == '__main__':
  test = realsensor()
  time.sleep(0.5)
  while not rospy.is_shutdown():
    try:
      test.healthChecker()
      test.pose_pub.publish(test.cur_pos)
      time.sleep(SLEEP_THRESHOLD) # caps CPU usage
    except KeyboardInterrupt:
      print("Shutting Down")
      break