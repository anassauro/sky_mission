import numpy as np 
import cv2

NUMTEST = 1

capture = cv2.VideoCapture(0)
testnum = NUMTEST
frame_width = int(capture.get(3))
frame_height = int(capture.get(4))
size = (frame_width, frame_height)
result = cv2.VideoWriter(f'video-test-{testnum}.avi', 
                cv2.VideoWriter_fourcc(*'MJPG'),
                10, size)

while capture.isOpened():
    ret, frame = capture.read()
    if ret == True:
        result.write(frame)
        cv2.waitKey(1) 
    
    
    else:
        break

print("Recorded")
capture.release()
result.release()
cv2.destroyAllWindows()
