import rospy 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import pyzbar.pyzbar as pyzbar
import re

"""def add_to_list_if_not_exists(mydata, data_list):
    if mydata not in data_list:
        data_list.append(mydata)
    return data_list
"""

def add_to_list_if_matches_pattern(mydata, data_list):
    pattern = r'[A-Da-d][0-4]'
    if re.match(pattern, mydata):
        if mydata not in data_list:
            data_list.append(mydata)        
    return data_list

def print_if_matches_pattern(mydata):
    pattern = r'[A-Da-d][0-4]'
    if re.match(pattern, mydata):
        print ("Impressao com filtro:", mydata)
    else:
        print ("Sem filtro:", mydata)

def adjust_brightness_contrast(frame, brightness=0, contrast=0):
    # Adjust brightness and contrast using numpy operations
    frame = np.int16(frame)
    frame = frame * (contrast / 127 + 1) - contrast + brightness
    frame = np.clip(frame, 0, 255)
    frame = np.uint8(frame)
    return frame

def detector(video_path):

    data_list = []  # Initialize an empty list to store unique mydata

    # Initialize the video capture
    video = cv2.VideoCapture(video_path)

    while(True):
      
        # Capture the video frame by frame
        ret, frame = video.read()
        
        fps = video.get(cv2.CAP_PROP_FPS)

        if not ret:
            break

        if fps > 0:
            cv2.waitKey(int(1000 / fps))

        # Adjust brightness and contrast
        frame = adjust_brightness_contrast(frame, brightness=30, contrast=70)

        # Find barcodes and Qr
        for barcode in pyzbar.decode(frame):
            
            # Convert the data from bytes to string
            mydata = barcode.data.decode('utf-8')
            print_if_matches_pattern(mydata) #if for pattern?
            
            # Add mydata to the list if it doesn't exist
            data_list = add_to_list_if_matches_pattern(mydata, data_list)
            
            # Add bounding box to qr codes and barcodes
            points = np.array([barcode.polygon], np.int32)
            points = points.reshape(-1, 1, 2)
            cv2.polylines(frame, [points], True, (255, 0, 0), 5) # draw a polygon on an image
            
            # Add text over the bounding box
            points2 = barcode.rect
            cv2.putText(frame, mydata, (points2[0], points2[1]), cv2.FONT_HERSHEY_SIMPLEX,
                      0.9, (255, 0, 0), 2)
  
        # Display the resulting frame
        cv2.imshow('frame', frame)
      
        # The 'q' button is set as the quitting button
        if cv2.waitKey(1) & 0xFF == ord('q'):
            video.release()
            cv2.destroyAllWindows()
            break

    # Release video and close window
    print(data_list)

# Main             
if __name__ == '__main__':
    # Pass the video file path to the detector function
    video_path = "captura13.mp4"  # Replace with the actual video file path
    detector(video_path)


class barcode_reader:
    def __init__(self):
        self.data_list = []
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/webcam/image_raw', Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            
        except CvBridgeError as e:
            print(e)
            
        else:

            self.detect(cv_image)
    
    def detect(self, image):
        #process image
        frame = adjust_brightness_contrast(image, brightness=30, contrast=70)
        
        for barcode in pyzbar.decode(frame):
            
            # Convert the data from bytes to string
            mydata = barcode.data.decode('utf-8')
            print_if_matches_pattern(mydata) #if for pattern?
            
            # Add mydata to the list if it doesn't exist
            self.data_list = add_to_list_if_matches_pattern(mydata, self.data_list)
            
            # Add bounding box to qr codes and barcodes
            points = np.array([barcode.polygon], np.int32)
            points = points.reshape(-1, 1, 2)
            cv2.polylines(frame, [points], True, (255, 0, 0), 5) # draw a polygon on an image
            
            # Add text over the bounding box
            points2 = barcode.rect
            cv2.putText(frame, mydata, (points2[0], points2[1]), cv2.FONT_HERSHEY_SIMPLEX,
                      0.9, (255, 0, 0), 2)
            
        print(self.data_list)
        
        cv2.imshow("image", frame)
        cv2.waitKey(1) & 0xFF   

def main():
    rospy.init_node('barcode_reader', anonymous=True)
    br = barcode_reader()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()