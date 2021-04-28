import cv2
import numpy as np

limits = {
    "red": (np.array([0, 10, 65]), np.array([20, 255, 255])),
    "yellow": (np.array([20, 10, 65]), np.array([40, 255, 255])),
    "green": (np.array([40, 10, 65]), np.array([70, 255, 255])),
    "blue": (np.array([70, 10, 65]), np.array([110, 255, 255])),
}


img = cv2.imread("/home/nik/ROS_ws/src/task_2/data/train_data/ring_img/train_img_3.png")

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower = np.array([20, 0, 0])
upper = np.array([255, 255, 255])
print(*limits["red"])
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, *limits["red"])
countour, hierarchy = cv2.findContours(
    mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
)

print(countour)

cv2.drawContours(img, countour, -1, (0, 255, 0), 2)
cont = max(countour, key=cv2.contourArea)
m = cv2.moments(cont)
cx = m["m10"] / m["m00"]
cy = m["m01"] / m["m00"]
cv2.circle(img, (int(cx), int(cy)), 3, (0, 255, 0))
print(cx, cy)


output = cv2.bitwise_and(img, img, mask=mask)


cv2.imshow("frame", img)
cv2.imshow("mask", mask)
cv2.imshow("res", output)

cv2.waitKey(0)
cv2.destroyAllWindows()