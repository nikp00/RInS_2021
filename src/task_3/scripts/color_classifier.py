import rospy
import os
import sys
from sklearn.neighbors import KNeighborsClassifier
import numpy as np
import pickle
from cv_bridge import CvBridge, CvBridgeError
import colorsys


from std_msgs.msg import ColorRGBA
from task_2.srv import ColorClassifierService, ColorClassifierServiceResponse


class ColorClassifier:
    def __init__(self, base_path):
        self.node = rospy.init_node("color_classifier")

        self.path = base_path
        self.bridge = CvBridge()
        self.load_data()

        self.srv = rospy.Service(
            "color_classifier", ColorClassifierService, self.predict
        )

        rospy.spin()

    def load_data(self):
        self.clf = pickle.load(open(self.path + "model.pickle", "rb"))
        self.color_definitions = pickle.load(open(self.path + "labels.pickle", "rb"))

    def predict(self, req):
        if req.mode == 0:
            img = self.bridge.imgmsg_to_cv2(req.image)
            x0, y0, x1, y1 = tuple(req.bounding_box)
            color = self.getColorFromImage(img, bounding_box=(x0, y0, x1, y1))
        elif req.mode == 1:
            rgb_value = (req.color.r, req.color.g, req.color.b)
            color = self.getColorNameFromRGB(rgb_value)
        else:
            return res
        res = ColorClassifierServiceResponse()
        res.color = color
        res.marker_color = ColorRGBA(*self.color_definitions[color], 1)
        return res

    def getColorNameFromRGB(self, RGBValue):
        hsv = colorsys.rgb_to_hsv(*[e / 255 for e in RGBValue])
        hsv = (hsv[0], 1, 1)
        return self.clf.predict([hsv])[0]

    def getColorFromImage(self, rgb_image, bounding_box=None, step=10):
        h = rgb_image.shape[0]
        w = rgb_image.shape[1]

        if bounding_box == None:
            bounding_box = (0, 0, w, h)

        x1, y1, x2, y2 = bounding_box

        colors = []
        for y in range(y1, y2, int((y2 - y1) / step)):
            for x in range(x1, x2, int((x2 - x1) / step)):
                b, g, r = rgb_image[y, x]
                colors.append(self.getColorNameFromRGB((r, g, b)))

        return max(set(colors), key=colors.count)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        base_path = sys.argv[1] + "color_classifier_data/"
    else:
        base_path = os.path.abspath(os.pardir) + "/data/color_classifier_data/"
    print(base_path)
    ColorClassifier(base_path)(base_path)
