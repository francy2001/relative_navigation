import cv2
import numpy as np
import pyrealsense2 as rs
import os

# --- CONFIGURAZIONE ---
# Dimensioni della scacchiera interna (numero di angoli interni)
chessboard_rows = 6  # righe interne
chessboard_cols = 9  # colonne interne
square_size = 0.025   # lato quadrato in metri (2.5 cm)

# Cartella dove salvare le immagini della scacchiera
output_folder = "chessboard_images"
os.makedirs(output_folder, exist_ok=True)

# --- PREPARAZIONE REALSENSE ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

# --- PREPARAZIONE PUNTI 3D DELLA SCACCHIERA ---
objp = np.zeros((chessboard_rows * chessboard_cols, 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_cols, 0:chessboard_rows].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # punti 3D
imgpoints = []  # punti 2D

img_counter = 0
try:
    print("Premi 'c' per catturare l'immagine della scacchiera, ESC per terminare")
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Rileva angoli della scacchiera
        ret, corners = cv2.findChessboardCorners(gray, (chessboard_cols, chessboard_rows), None)

        if ret:
            # Migliora precisione
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), 
                                        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            cv2.drawChessboardCorners(frame, (chessboard_cols, chessboard_rows), corners2, ret)

        cv2.imshow('RealSense Chessboard', frame)
        key = cv2.waitKey(1)

        if key & 0xFF == ord('c') and ret:  # salva solo se angoli trovati
            img_filename = os.path.join(output_folder, f"img_{img_counter:03d}.png")
            cv2.imwrite(img_filename, frame)
            imgpoints.append(corners2)
            objpoints.append(objp)
            img_counter += 1
            print(f"[INFO] Saved {img_filename}")
        elif key & 0xFF == 27:  # ESC per uscire
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

# --- CALIBRAZIONE DELLA CAMERA ---
if len(objpoints) < 5:
    print("Errore: cattura almeno 5 immagini valide della scacchiera.")
else:
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("\n--- RISULTATI CALIBRAZIONE ---")
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion Coefficients:\n", dist_coeffs.ravel())

    # Salva su file per riutilizzo
    np.save("camera_matrix.npy", camera_matrix)
    np.save("dist_coeffs.npy", dist_coeffs)
    print("\nParametri salvati su 'camera_matrix.npy' e 'dist_coeffs.npy'")
