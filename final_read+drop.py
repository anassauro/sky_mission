import rospy
import time
import numpy as np
import RPi.GPIO as GPIO
from std_msgs.msg import String

from mav import MAV2

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
    x, y, z = 1, 1, 1.5
    base_a = [4, 2, 1.5]
    base_c = [0, 1, 1.5]
    tempo = time.time()

    electromagnet = 11
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(electromagnet, GPIO.OUT)
    GPIO.output(electromagnet, True)

    while True:
        try:
            dr.takeoff(z)
            rospy.sleep(7)
            dr.go_to_local([x, y, z])  
            rospy.sleep(7)
            
            while (z > 0.4 and not qr.found_qr) or (time.time() - tempo >= 20):
                dr.go_to_local([x, y, z])
                rospy.sleep(2)
                z -= 0.1
                    
            if not qr.found_qr:
                print("Not found")
                rospy.sleep(2)
                dr.go_to_local([x, y, 1.5])
                GPIO.output(electromagnet, False)
                print("soltei a carga")
                rospy.sleep(2)
                dr.go_to_local([0, 0, 1.5])
                rospy.sleep(2)
                dr.land()

            else:
                print("QR Code found!")
                print("QR Code list:", qr.qrcode_list)
                if (qr.qrcode_list[0] == "A"):
                    dr.go_to_local(base_a)
                    rospy.sleep(2)
                    GPIO.output(electromagnet, False)
                    print("soltei a carga")

                if (qr.qrcode_list[0] == "C"):
                    dr.go_to_local(base_c)
                    rospy.sleep(2)
                    GPIO.output(electromagnet, False)
                    print("soltei a carga")



            dr.go_to_local([x, y, 1.5])
            rospy.sleep(3)
            dr.go_to_local([0, 0, 1.5])
            print("cheguei na base")
            rospy.sleep(2)
            dr.land()
            print ("pousei kk")

                
        except KeyboardInterrupt:
            print("fim")       

main()



