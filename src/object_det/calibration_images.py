import pyrealsense2 as rs
import numpy as np
import cv2
import os

# Cartella dove salvare le immagini
output_folder = "chessboard_images"
os.makedirs(output_folder, exist_ok=True)

# Configura il pipeline RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Avvia la pipeline
pipeline.start(config)

try:
    img_counter = 0
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Converti a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Mostra immagine
        cv2.imshow('RealSense RGB', color_image)
        key = cv2.waitKey(1)

        if key & 0xFF == ord('c'):  # premi 'c' per catturare
            filename = os.path.join(output_folder, f"img_{img_counter:03d}.png")
            cv2.imwrite(filename, color_image)
            print(f"[INFO] Saved {filename}")
            img_counter += 1
        elif key & 0xFF == 27:  # premi ESC per uscire
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
