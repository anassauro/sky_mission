import rospy
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget, StatusText
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Image
import time
class PeopleDetection:

    def __init__(self):

        rospy.init_node('status_text_publisher', anonymous=True)
        self.rate = rospy.Rate(1)  # 1 Hz
        # Subscribers

        # Publishers
        self.text_pub = rospy.Publisher('/mavros/statustext/send', StatusText, queue_size=10)

    def send_text(self):

        while True:
            status_msg = StatusText()
            status_msg.header.stamp = rospy.Time.now()
            status_msg.severity = StatusText.INFO  # You can change this to your desired severity
            status_msg.text = "Tico tico no fuba"

            self.text_pub.publish(status_msg)
            self.rate.sleep()
if __name__ == '__main__':
    try:
        detection = PeopleDetection()
        detection.send_text()
        
    except rospy.ROSInterruptException:
        pass