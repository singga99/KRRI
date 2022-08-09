import cv2
from matplotlib import use
import numpy as np
import yaml

cap = cv2.VideoCapture(0)

w = 1920
h = 1080
    
f = open("/home/krri/shin_dev/cam_calib/tmp/calibrationdata/ost.yaml")
cal_mat = yaml.load(f, Loader = yaml.FullLoader)

mtx = cal_mat['camera_matrix']
cm_data = mtx['data']

dist = cal_mat['distortion_coefficients']
dist_data = dist['data']

cameraMatrix = np.array(cm_data).reshape(3, 3)
distCoeffs = np.array(dist_data).astype(np.float64)
dist_coeffs = np.zeros((4, 1))

points_2D = np.array([(760, 410),   # 좌 상단
                      (1101, 389),  # 우 상단
                      (767, 765),   # 좌 하단
                      (1130, 728),  # 우 하단
                      (716, 432),   # 옷걸이
                      (683, 684)], dtype = "double")    # 교통 표지판

points_3D = np.array([(-1.4, -0.42, 0.17),  # 좌 상단
                      (-1.5, 0.4, 0.18),    # 우 상단
                      (-1.3, -0.39, -0.38), # 좌 하단
                      (-1.5, 0.4, -0.36),   # 우 하단
                      (-4.5, -1.4, 0.89),   # 옷걸이
                      (-3.5, -1.5, -0.58)], dtype = "double")   # 교통 표지판

retval, rvec, tvec = cv2.solvePnP(points_3D, points_2D, cameraMatrix, dist_coeffs, flags=0)

R = cv2.Rodrigues(rvec)
t = tvec

print(R, "\n")
print(t)

# while True:
#     ret, frame = cap.read()
    
#     newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (w, h), 0, (w, h))
#     dst = cv2.undistort(frame, cameraMatrix, distCoeffs, None, newcameramtx)
    
#     if ret:
#         cv2.imshow("camera", dst)
        
#         if cv2.waitKey(1) & 0xFF == 27: # esc 키를 누르면 닫음
#             break

#     else:
#         break
    
# cap.release()
# cv2.destroyAllWindows()
