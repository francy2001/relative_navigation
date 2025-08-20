import random
import cv2
import numpy as np
import os

os.makedirs("markers", exist_ok=True)

single_marker = False
aruco_type = cv2.aruco.DICT_6X6_250

num_markers = 5  
page_size = (2480, 3508)  # A4 size in pixels at 300 DPI
marker_size = 500  

dictionary = cv2.aruco.getPredefinedDictionary(aruco_type)
page = np.ones((page_size[0], page_size[1], 3), dtype=np.uint8) * 255  # white board

used_positions = []

if single_marker:
    marker_id = random.randint(0, dictionary.bytesList.shape[0] - 1)
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size)
    cv2.imwrite("markers/single_random_marker.png", marker_img)
    exit(0)

for i in range(num_markers):
    marker_id = random.randint(0, dictionary.bytesList.shape[0] - 1)
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size)
    print(f"Generated marker ID: {marker_id}")

    while True:
        # Setting the marker position randomly inside the board
        x = random.randint(0, page_size[1] - marker_size)
        y = random.randint(0, page_size[0] - marker_size)
        # Check if the position is too close to existing markers
        too_close = any(abs(px - x) < marker_size and abs(py - y) < marker_size
                        for px, py in used_positions)
        if not too_close:
            used_positions.append((x, y))
            break

    page[y:y+marker_size, x:x+marker_size] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

cv2.imwrite("markers/multiple_random_markers.png", page)
cv2.imshow("Markers Page", page)
cv2.waitKey(0)
cv2.destroyAllWindows()
