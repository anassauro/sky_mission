import cv2
import cv2.aruco as aruco

# Initialize the video capture object
cap = cv2.VideoCapture(0)  # Use 0 for the default camera. Change if you have multiple cameras.

# Load the ArUco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)  # Choose the desired dictionary size here

# Create parameters for ArUco detection
parameters = aruco.DetectorParameters_create()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if ret:
        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) > 0:
            # Print detected marker IDs
            print(f"Detected ArUco IDs: {ids}")

            # Draw detected markers on the frame
            aruco.drawDetectedMarkers(frame, corners, ids)

        # Display the frame
        cv2.imshow('Frame', frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
