from mav import MAV2
import rospy
from std_msgs.msg import String
import time

YOLO_TRIES = 3
WAIT_FOR_REALSENSE = 5
TAKEOFF_DIST = 2
TIME_LIMIT = 2

rospy.init_node('stage_1')

dr = MAV2()

def centralize(dr : MAV2):
    
    pass



#the place the drone will go so it can start detecting things

search_points = ((1, 1, 2), (-1, -1, 2), (-1, -1, 2), (-1, -1, 2),)

serv = rospy.ServiceProxy('pad_service', String)
"""
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
"""

dr = MAV2()

for point in search_points:
    dr.go_to_local((point[0], point[1], point[2]))

    rospy.wait_for_service("pad_service")
    for _ in range(YOLO_TRIES):
        request = serv("Chefia naum liberou o aumosso")
        time.sleep(5)
        if(request.data == "Pad Found!"):

            time.sleep(WAIT_FOR_REALSENSE)

            #centralize(dr)
            dr.land()

            time.sleep(10)

            dr.takeoff(TAKEOFF_DIST)

            break
        else:
            print("Pad not found. Trying again...")


dr.go_to_local((0, 0, TAKEOFF_DIST))

dr.land()


