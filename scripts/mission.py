from sky_mission.sky_utils.communication import Mav
import rospy

def main():
    mav = Mav(debug=True)

    rospy.loginfo("Taking off...")

    if not mav.takeoff(5):
        print("Couldn't takeoff.")
    else:
        rospy.sleep(6)
    
    rospy.loginfo("Trying to land...")
    if not mav.land():
        rospy.loginfo("Couldnt't land.")

if __name__ == "__main__":
    try:
        main()
    except:
        pass