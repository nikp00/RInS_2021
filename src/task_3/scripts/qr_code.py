#!/usr/bin/python3
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import ColorRGBA

import pyzbar.pyzbar as pyzbar

dictm = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# The object that we will pass to the markerDetect function
params = cv2.aruco.DetectorParameters_create()

print(params.adaptiveThreshConstant)
print(params.adaptiveThreshWinSizeMax)
print(params.adaptiveThreshWinSizeMin)
print(params.minCornerDistanceRate)
print(params.adaptiveThreshWinSizeStep)

params.adaptiveThreshConstant = 25
adaptiveThreshWinSizeStep = 2


class QRExtractor:
    def __init__(self):
        rospy.init_node("image_converter", anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the image and/or depth topic
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_callback
        )

    def image_callback(self, data):
        # print('Iam here!')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

            # Find a QR code in the image
        decodedObjects = pyzbar.decode(cv_image)

        # print(decodedObjects)

        if len(decodedObjects) == 1:
            dObject = decodedObjects[0]
            print("Found 1 QR code in the image!")
            print("Data: ", dObject.data, "\n")

            # Visualize the detected QR code in the image
            points = dObject.polygon
            if len(points) > 4:
                hull = cv2.convexHull(
                    np.array([point for point in points], dtype=np.float32)
                )
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points

            ## Number of points in the convex hull
            n = len(hull)

            ## Draw the convext hull
            for j in range(0, n):
                cv2.line(cv_image, hull[j], hull[(j + 1) % n], (0, 255, 0), 2)

            cv2.imshow("Warped image", cv_image)
            cv2.waitKey(1)

        elif len(decodedObjects) == 0:
            print("No QR code in the image")
        else:
            print("Found more than 1 QR code")


def main(args):

    qre = QRExtractor()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)