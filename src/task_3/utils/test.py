import numpy as np
import cv2

img_c = cv2.imread("/home/nik/ROS_ws/src/task_2/data/train_data/ring_img/img_15.png")

img = cv2.cvtColor(img_c, cv2.COLOR_BGR2GRAY)


img = cv2.equalizeHist(img)
ret, thresh = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)

contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

elps = []
for cnt in contours:
    if cnt.shape[0] >= 20:
        ellipse = cv2.fitEllipse(cnt)
        elps.append(ellipse)

candidates = []
for n in range(len(elps)):
    for m in range(n + 1, len(elps)):
        e1 = elps[n]
        e2 = elps[m]
        dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
        if dist < 5:
            candidates.append((e1, e2))

skip = True
for c in candidates:
    e1 = c[0]
    e2 = c[1]

    size = (e1[1][0] + e1[1][1]) / 2
    center = (e1[0][1], e1[0][0])

    if center[0] > img_c.shape[0] / 3:
        continue

    skip = False
    x1 = int(center[0] - size / 2)
    x2 = int(center[0] + size / 2)

    img_cc = cv2.copyTo(img_c, None)

    cv2.ellipse(img_c, e1, color=(255, 0, 0))
    cv2.ellipse(img_c, e2, color=(255, 0, 0))
    cv2.circle(img_c, (int(center[1]), int(center[0])), 3, (0, 255, 0), 3)

    m1 = np.zeros(img_c.shape[0:2], np.uint8)
    m2 = np.zeros(img_c.shape[0:2], np.uint8)

    # cv2.ellipse(m1, e1, (255, 255, 255), -1)
    # cv2.ellipse(m2, e2, (255, 255, 255), -1)
    cv2.ellipse(m1, e1, (255, 255, 255), -1)
    cv2.ellipse(m2, e2, (255, 255, 255), -1)
    m3 = m2 - m1

    print(cv2.mean(img_cc, mask=m3))

    img_cc = cv2.bitwise_and(img_cc, img_cc, mask=m3)
    # cv2.floodFill(img_c, None, (int(center[1]), int(center[0])), (0, 0, 255))
    print("asds")

cv2.imshow("asd", img_c)
cv2.imshow("m1", m1)
cv2.imshow("m2", m2)
cv2.imshow("m3", m3)
cv2.imshow("cc", img_cc)
cv2.waitKey(0)
