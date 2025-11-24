import cv2
import cv2.aruco as aruco
import numpy as np

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

for marker_id in range(16):
    img = aruco.drawMarker(aruco_dict, marker_id, 200)
    img_with_border = np.ones((240, 240), dtype=np.uint8) * 255
    img_with_border[20:220, 20:220] = img
    cv2.imwrite(f'aruco_{marker_id}.png', img_with_border)
    print(f"Generated aruco_{marker_id}.png")
