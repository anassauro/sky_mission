import rospy
from std_msgs.msg import String


rospy.init_node('MAMAS')

serv = rospy.ServiceProxy('pad_service', String)
rospy.wait_for_service("pad_service")

while(True):
    try:
        input()
        request = serv("AUMOSSAR")
        if(request.data == "Pad Found!"):
            print("NAO MAMAMOS")
        else:
            print("MAMAMOS")
        print("")

    except KeyboardInterrupt:
        break