import rospy
import time
import math
import time

#from actuator import Actuator
from std_msgs.msg import String
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
