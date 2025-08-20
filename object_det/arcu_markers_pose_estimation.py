import time
import cv2
import cv2.aruco as aruco
import numpy as np

from scipy.spatial.transform import Rotation as R

aruco_type = cv2.aruco.DICT_6X6_250

# Setting intrinsics parameters of the camera (taken from calibration)
camera_matrix = np.load("camera_matrix.npy")
dist_coeffs = np.load("dist_coeffs.npy")

print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)

cap = cv2.VideoCapture(35)
time.sleep(2)  # Allow camera to warm up

aruco_dict = aruco.getPredefinedDictionary(aruco_type)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# Frame reading loop
while True:
    ret, frame = cap.read()
    if not ret:
        break  

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Markers detection
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Pose estimation for each detected marker
        rvec, tvec = [], []
        for i in range(len(ids)):
            markerLength = 0.05  # 5 cm
            rvec_i, tvec_i, _ = aruco.estimatePoseSingleMarkers(corners[i], markerLength, camera_matrix, dist_coeffs)
            rvec.append(rvec_i[0])
            tvec.append(tvec_i[0])
            
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec[i], tvec[i], 0.1)

            # Print Id and translation/rotation vectors --> relative coordinates of the marker with respect to the camera
            print(f"Marker ID: {ids[i][0]}, tvec: {tvec}, rvec: {rvec}")
    
        # Pose estimation for the object where the markers are placed --> simple mean of the markers' positions
        if len(tvec) > 0:
            t_fused = np.mean(np.array(tvec), axis=0)
            quaternions = [R.from_rotvec(r).as_quat() for r in rvec]
            quat_mean = np.mean(quaternions, axis=0)
            quat_mean /= np.linalg.norm(quat_mean)  # normalizza
            r_fused = R.from_quat(quat_mean).as_rotvec()

            print(f"Fused tvec: {t_fused}, Fused rvec: {r_fused}")

    cv2.imshow('frame', frame)

    # Quit with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
