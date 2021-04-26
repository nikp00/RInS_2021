import sys
import os
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import ColorRGBA

from pynput.keyboard import Listener


class ImageCapture:
    def __init__(self, path):
        self.node = rospy.init_node("capture_train_img")
        self.bridge = CvBridge()
        self.img_subscriber = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_callback
        )
        self.path = path
        self.seq = 0
        self.image = None
        self.key = False
        self.listener = Listener(on_press=self.key_press, on_release=self.key_release)
        self.listener.start()
        self.run()

    def run(self):
        print("Started")
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.key:
                cv2.imwrite(f"{self.path}/train_img_{self.seq}.png", self.image)
                print(f"Saved img: {self.seq}")
                self.seq += 1
                self.key = False
            r.sleep()

    def key_press(self, key):
        self.key = key.name == "space"

    def key_release(self, key):
        self.key = False

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    print(sys.argv)
    ImageCapture(sys.argv[1])