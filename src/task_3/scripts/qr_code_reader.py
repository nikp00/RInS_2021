#!/usr/bin/python3
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
import pyzbar.pyzbar as pyzbar

from task_3.srv import QRCodeReaderService, QRCodeReaderServiceResponse
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA


class QRCodeReader:
    def __init__(self):
        self.node = rospy.init_node("qr_code_reader")
        self.srv = rospy.Service("qr_code_reader", QRCodeReaderService, self.decode)
        self.bridge = CvBridge()
        rospy.spin()

    def decode(self, req):
        try:
            image = self.bridge.imgmsg_to_cv2(
                rospy.wait_for_message("/camera/rgb/image_raw", Image), "bgr8"
            )
        except CvBridgeError as e:
            print(e)

        mask = cv2.inRange(image, (0, 0, 0), (200, 200, 200))
        thresholded = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        inverted = 255 - thresholded  # black-in-white

        decodedObjects = pyzbar.decode(inverted)

        if len(decodedObjects) == 1:
            dObject = decodedObjects[0]
            print("Found 1 QR code in the image!")
            print("Data: ", dObject.data, "\n")

            # # Visualize the detected QR code in the image
            # points = dObject.polygon
            # if len(points) > 4:
            #     hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            #     hull = list(map(tuple, np.squeeze(hull)))
            # else:
            #     hull = points

            # ## Number of points in the convex hull
            # n = len(hull)

            # ## Draw the convext hull
            # for j in range(0, n):
            #     cv2.line(inverted, hull[j], hull[(j + 1) % n], (0, 255, 0), 2)

            # cv2.imshow("Warped image", inverted)
            # cv2.waitKey(1)
        else:
            print("No qr code detected")

        return QRCodeReaderServiceResponse("", 0)


if __name__ == "__main__":
    QRCodeReader()