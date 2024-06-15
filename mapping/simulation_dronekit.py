import dronekit
from dronekit import connect, VehicleMode
import cv2
import time
from fractions import Fraction
from PIL import Image as img
import piexif
from datetime import datetime
import argparse

class Vehicle():

    def __init__(self):

        self.mission = Mission()
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default = 'tcp:127.0.0.1:5763')
        args = parser.parse_args()

        #-- Connect to the vehicle
        print('Connecting...')
        self.vehicle = connect(args.connect)

        #-- Check vehicle status
        print(f"Mode: {self.vehicle.mode.name}")
        print(" Global Location: %s" % self.vehicle.location.global_frame)
        print(" Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)
        print(" Local Location: %s" % self.vehicle.location.local_frame)
        print(" Attitude: %s" % self.vehicle.attitude)
        print(" Velocity: %s" % self.vehicle.velocity)
        print(" Gimbal status: %s" % self.vehicle.gimbal)
        print(" EKF OK?: %s" % self.vehicle.ekf_ok)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Rangefinder: %s" % self.vehicle.rangefinder)
        print(" Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        print(" Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Armed: %s" % self.vehicle.armed)    # settable

        @self.vehicle.on_message('GLOBAL_POSITION_INT')
        def listener(_, name, message):
            print("GPS lat: %f" % (message.lat / 10000000))
            print("GPS lon: %f" % (message.lon / 10000000))
            self.mission.save_pictures(message)
        
        @self.vehicle.on_message('MISSION_CURRENT')
        def waypoint_listener(_, name, message):
            print("Reached waypoint %d" % message.seq)
            # Check if the received waypoint sequence matches the current waypoint index
            if(message.seq == 58):
                print("Beginning precision landing...")
                
class Mission():

    def __init__(self):
        self.quantidade_fotos = 0
        self.cam_frame = None
        self.capture = cv2.VideoCapture(2)


    def save_pictures(self, message):
        self.cam_frame = self.capture.read()[1]
        self.cam_frame = cv2.resize(self.cam_frame, (960, 540))
        name = "/home/helena/26AGOSTO2023/simulation_images/image%d.jpg" % self.quantidade_fotos
        # output = "/home//Documents/tagged/image%d.jpg" % self.quantidade_fotos
        name_clean = "image%d.jpg" % self.quantidade_fotos
        latitude = message.lat / 10000000  # Convert from degrees * 1e7 to degrees
        longitude = message.lon / 10000000  # Convert from degrees * 1e7 to degrees

        cv2.imwrite(name, self.cam_frame)
        self.add_gps_metadata(name, latitude, longitude)

        print("Image {} at lat: {}, long: {}".format(self.quantidade_fotos, latitude, longitude))
        self.quantidade_fotos += 1
    
    def float_to_rational(self, f):
        f = Fraction(f).limit_denominator()
        return f.numerator, f.denominator

    def add_gps_metadata(self, image_path, latitude, longitude):
        exif_dict = piexif.load(image_path)

        lat_deg = abs(latitude)
        lat_min, lat_sec = divmod(lat_deg * 3600, 60)
        lat_deg, lat_min = divmod(lat_min, 60)
        lat_ref = 'N' if latitude >= 0 else 'S'

        lon_deg = abs(longitude)
        lon_min, lon_sec = divmod(lon_deg * 3600, 60)
        lon_deg, lon_min = divmod(lon_min, 60)
        lon_ref = 'E' if longitude >= 0 else 'W'

        lat_deg_num, lat_deg_den = self.float_to_rational(lat_deg)
        lat_min_num, lat_min_den = self.float_to_rational(lat_min)
        lat_sec_num, lat_sec_den = self.float_to_rational(lat_sec)

        lon_deg_num, lon_deg_den = self.float_to_rational(lon_deg)
        lon_min_num, lon_min_den = self.float_to_rational(lon_min)
        lon_sec_num, lon_sec_den = self.float_to_rational(lon_sec)

        gps_ifd = {
            piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
            piexif.GPSIFD.GPSLatitudeRef: lat_ref,
            piexif.GPSIFD.GPSLatitude: ((lat_deg_num, lat_deg_den), (lat_min_num, lat_min_den), (lat_sec_num, lat_sec_den)),
            piexif.GPSIFD.GPSLongitudeRef: lon_ref,
            piexif.GPSIFD.GPSLongitude: ((lon_deg_num, lon_deg_den), (lon_min_num, lon_min_den), (lon_sec_num, lon_sec_den)),
        }
        
        exif_dict['GPS'] = gps_ifd

        timestamp = datetime.now()
        timestamp_str = timestamp.strftime("%Y:%m:%d %H:%M:%S")

        exif_dict['0th'][piexif.ImageIFD.DateTime] = timestamp_str
        exif_dict['Exif'][piexif.ExifIFD.DateTimeOriginal] = timestamp_str
        exif_dict['Exif'][piexif.ExifIFD.DateTimeDigitized] = timestamp_str

        exif_bytes = piexif.dump(exif_dict)
        piexif.insert(exif_bytes, image_path)

        print("GPS metadata added to the image.")

if __name__ == '__main__':
        
    mapping = Vehicle()


    while True:
        pass
