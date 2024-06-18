import cv2
import numpy as np
from cv2 import aruco

class MarkerDetector:

    def __init__(self, target_type, target_size, camera_info):
        self.target_type = target_type
        self.marker_size = target_size

        if self.target_type == 'aruco':
            self.dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
            self.parameters = aruco.DetectorParameters_create()
        elif self.target_type == 'qrcode':
            print("QR Code not implemented yet!")

        self.camera_matrix = camera_info[0]
        self.dist_coeff = camera_info[1]

        self.np_camera_matrix = np.array(self.camera_matrix)
        self.np_dist_coeff = np.array(self.dist_coeff)

        self.horizontal_res = camera_info[2][0]
        self.vertical_res = camera_info[2][1]

        self.horizontal_fov = camera_info[3][0]
        self.vertical_fov = camera_info[3][1]

    def pose_estimation(self, corners, marker_size, mtx, distortion):
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, -marker_size / 2, 0],
                                  [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        nada, rvec, tvec = cv2.solvePnP(marker_points, corners, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        return rvec, tvec

    def aruco_detection(self, frame):
        markerCorners, markerIds, rejected = aruco.detectMarkers(
            frame, self.dictionary, parameters=self.parameters
        )

        if markerIds is not None and len(markerCorners) > 0:
            closest_target = []
            closest_dist = float('inf')

            for i, corners in enumerate(markerCorners):
                marker_points = corners[0]
                final_image = frame

                try:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        [marker_points], self.marker_size, self.np_camera_matrix, self.np_dist_coeff
                    )

                    x = round(tvec[0][0][0], 2)
                    y = round(tvec[0][0][1], 2)
                    z = round(tvec[0][0][2], 2)

                    x_avg = np.mean(marker_points[:, 0])
                    y_avg = np.mean(marker_points[:, 1])

                    x_ang = (x_avg - self.horizontal_res * 0.5) * self.horizontal_fov / self.horizontal_res
                    y_ang = (y_avg - self.vertical_res * 0.5) * self.vertical_fov / self.vertical_res

                    payload = markerIds[i][0]

                    if z < closest_dist:
                        closest_dist = z
                        closest_target = [x, y, z, x_ang, y_ang, payload, marker_points]
                except Exception as e:
                    print(f"Error in pose estimation: {e}")

            return closest_target
        return None

    def draw_marker(self, frame, points):
        topLeft, topRight, bottomRight, bottomLeft = points

        tR = (int(topRight[0]), int(topRight[1]))
        bR = (int(bottomRight[0]), int(bottomRight[1]))
        bL = (int(bottomLeft[0]), int(bottomLeft[1]))
        tL = (int(topLeft[0]), int(topLeft[1]))

        cX = int((tR[0] + bL[0]) / 2.0)
        cY = int((tR[1] + bL[1]) / 2.0)

        rect = cv2.rectangle(frame, tL, bR, (0, 0, 255), 2)
        final = cv2.circle(rect, (cX, cY), radius=4, color=(0, 0, 255), thickness=-1)

        return final

def main():
    # Camera parameters
    camera_matrix = [
        [629.60088304, 0.0, 649.96450783],
        [0.0, 628.99975883, 323.37037351],
        [0.0, 0.0, 1.0]
    ]
    dist_coeff = [-0.08654266, 0.00064634, -0.01367921, 0.00537603, 0.00417901]

    # ArUco marker parameters
    marker_type = 'aruco'
    marker_size = 0.20  # 35 cm in meters

    camera_info = [camera_matrix, dist_coeff, (1280, 720), (1.58717, 1.03966)]
    detector = MarkerDetector(marker_type, marker_size, camera_info)

    cap = cv2.VideoCapture(2)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        closest_target = detector.aruco_detection(frame)

        if closest_target is not None:
            x, y, z, x_ang, y_ang, payload, marker_points = closest_target
            print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')
            frame = detector.draw_marker(frame, marker_points)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
