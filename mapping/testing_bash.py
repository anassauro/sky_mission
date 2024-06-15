
import cv2
import rospy
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Image
from cv_bridge import CvBridge 
import numpy as np
import time
import sys
from fractions import Fraction
from PIL import Image as img
import piexif
from PIL.ExifTags import TAGS, GPSTAGS
from decimal import Decimal, getcontext
from datetime import datetime
import subprocess
import os
#cap = cv2.VideoCapture(0)

class Mapping():

    def __init__(self):

        self.bridge_object = CvBridge()
        rospy.init_node('mapping_node', anonymous=False)
        self.quantidade_fotos = 0
        self.rate = rospy.Rate(0.769)
        self.global_pose = NavSatFix()
        # self.capture = cv2.VideoCapture(0)
        self.global_position_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_callback)
        #self.cam_sub = rospy.Subscriber('/webcam/image_raw', Image, self.cam_callback)
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
       # self.image_dir = os.path.join(self.script_dir, "images")


    
    def global_callback(self, message):
        self.global_pose = message
        # succes, self.cam_frame = self.capture.read()                  
        # self.cam_frame = cv2.resize(self.cam_frame, (960,540))
        print("cheguei")
        #frame = self.cam_frame
        #print(frame)
        #cv2.imshow("frame")
        rospy.loginfo("Message received")
        self.save_pictures()
        
    def save_pictures(self):

        #print("cheguei")
        longitude = self.global_pose.longitude
        latitude = self.global_pose.latitude
        name = self.script_dir+"/images/oficial%d.jpg"%self.quantidade_fotos
        process = subprocess.Popen(['raspistill','-ISO', '400','-ss','1000','--metering', 'average','--timeout','2000','-o', name], stdout=subprocess.PIPE, text=True)
        # other methods: raspistill --width 1280 --height 960 --quality 100 --timeout 500 --encoding jpg -sh 0 -co 0 -br 50 -sa 0 -ev 0 --exposure sports --awb sun --metering average --nopreview --output $PROJECTDIR/$IMAGEDIR/$filename
        while True:
            output = process.stdout.readline()
            if output:
                  print(output.strip())
            result = process.poll()
            if result is not None:
                break

        self.add_gps_metadata(name,latitude, longitude)
        rospy.loginfo("Image " + str(self.quantidade_fotos) + " at lat: " +  str(latitude) + ", long: " + str(self.global_pose.longitude))
        self.quantidade_fotos += 1
        
    
    def float_to_rational(self,f):
        """
        Convert a floating-point number to a rational number (numerator, denominator).
        """
        f = Fraction(f).limit_denominator()
        return f.numerator, f.denominator

    def add_gps_metadata(self,image_path, latitude, longitude):
        exif_dict = piexif.load(image_path)

        # Convert latitude and longitude to degrees, minutes, and seconds
        lat_deg = abs(latitude)
        lat_min, lat_sec = divmod(lat_deg * 3600, 60)
        lat_deg, lat_min = divmod(lat_min, 60)
        lat_ref = 'N' if latitude >= 0 else 'S'

        lon_deg = abs(longitude)
        lon_min, lon_sec = divmod(lon_deg * 3600, 60)
        lon_deg, lon_min = divmod(lon_min, 60)
        lon_ref = 'E' if longitude >= 0 else 'W'

        # Convert coordinates to rational values
        lat_deg_num, lat_deg_den = self.float_to_rational(lat_deg)
        lat_min_num, lat_min_den = self.float_to_rational(lat_min)
        lat_sec_num, lat_sec_den = self.float_to_rational(lat_sec)

        lon_deg_num, lon_deg_den = self.float_to_rational(lon_deg)
        lon_min_num, lon_min_den = self.float_to_rational(lon_min)
        lon_sec_num, lon_sec_den = self.float_to_rational(lon_sec)

        # Construct the GPS coordinates
        gps_ifd = {
            piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
            piexif.GPSIFD.GPSLatitudeRef: lat_ref,
            piexif.GPSIFD.GPSLatitude: ((lat_deg_num, lat_deg_den), (lat_min_num, lat_min_den), (lat_sec_num, lat_sec_den)),
            piexif.GPSIFD.GPSLongitudeRef: lon_ref,
            piexif.GPSIFD.GPSLongitude: ((lon_deg_num, lon_deg_den), (lon_min_num, lon_min_den), (lon_sec_num, lon_sec_den)),
        }
        
        exif_dict['GPS'] = gps_ifd
        # Update the image's EXIF data with the GPS information
        # Get the current date and time
        timestamp = datetime.now()
        timestamp_str = timestamp.strftime("%Y:%m:%d %H:%M:%S")

    # Construct the time metadata
        exif_dict['0th'][piexif.ImageIFD.DateTime] = timestamp_str
        exif_dict['Exif'][piexif.ExifIFD.DateTimeOriginal] = timestamp_str
        exif_dict['Exif'][piexif.ExifIFD.DateTimeDigitized] = timestamp_str
        # Convert the EXIF data back to bytes
        exif_bytes = piexif.dump(exif_dict)

        # Save the updated image with the GPS metadata
        piexif.insert(exif_bytes, image_path)

        print("GPS metadata added to the image.")

if __name__ == '__main__':

    mapping = Mapping()

    while not rospy.is_shutdown():
        rospy.spin()
       

   
