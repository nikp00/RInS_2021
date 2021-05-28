import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import ColorRGBA
import pytesseract


class DigitExtractor:
    def __init__(self):
        self.node = rospy.init_node("digit_extractor")

        self.bridge = CvBridge()

        pass


if __name__ == "__main__":
    DigitExtractor()