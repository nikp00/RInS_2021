#!/usr/bin/python3
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
import pyzbar.pyzbar as pyzbar

from task_3.srv import QRCodeReaderService, QRCodeReaderServiceResponse
from sensor_msgs.msg import Image


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
        threshold = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        inverted = 255 - threshold  # black-in-white

        decodedObjects = pyzbar.decode(inverted)

        if len(decodedObjects) == 1:
            dObject = decodedObjects[0]
            print("Found 1 QR code in the image!")
            print("Data: ", str(dObject.data), "\n")
            return QRCodeReaderServiceResponse(str(dObject.data), 0)
        else:
            print("No QR code found!")
            return QRCodeReaderServiceResponse("", 1)


if __name__ == "__main__":
    QRCodeReader()