from turtle import shape
import cv2
import numpy as np


def mouse_handler(event, x, y, flags, data):
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(data['im'], (x, y), 3, (0, 0, 255), 5, 16)
        cv2.imshow('img', data['im'])
        print(x,y)
        
        if len(data['points']) < 4:
            data['points'].append([x, y])
    
def get_four_points(im):
    data = {}
    data['im'] = im.copy()
    data['points'] = []
    
    cv2.imshow('img', im)
    cv2.setMouseCallback('img', mouse_handler, data)
    cv2.waitKey(0)
    
    points = np.array(data['points'], dtype=float)
    
    return points
    

rgb_img = cv2.imread('/home/krri/shin_dev/vis_camera/extract/rgb/3.jpg')

dst_size = (400, 300, 3)
img_dst = np.zeros(dst_size, np.uint8)

rgb_dst = get_four_points(rgb_img)

points_dst = np.array([0, 0,
                       dst_size[1], 0,
                       dst_size[1], dst_size[0],
                       0, dst_size[0]], dtype=float)

points_dst = points_dst.reshape(4,2)
h, status = cv2.findHomography(rgb_dst, points_dst)
img_dst = cv2.warpPerspective(rgb_img, h, (dst_size[1],dst_size[0]))

cv2.waitKey(0)
cv2.destroyAllWindows()

