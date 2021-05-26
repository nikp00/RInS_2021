# pip3 install -U scikit-learn

from timeit import default_timer as timer

import rospy
import csv
import cv2
from sklearn import preprocessing
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from task_3.srv import color

# https://medium.com/analytics-vidhya/building-rgb-color-classifier-part-1-af58e3bcfef7
# https://github.com/AjinkyaChavan9/RGB-Color-Classifier-with-Deep-Learning-using-Keras-and-Tensorflow/blob/master/Dataset/final_data.csv


class colorDetector:

    # path = pot do dataseta, k = stevilo sosedov za knn (default = 3)
    def __init__(self, path, k=3):
        self.data = []
        self.labels = []

        # preberemo dataset iz csv v tabeli data in labels
        self.importDataset(path)

        self.model = KNeighborsClassifier(k)
        self.model.fit(self.data, self.labels)

    def getColorNameFromRGB(self, RGBValue):
        return self.model.predict([list(RGBValue)])[0]

    # Bounding box je tuple (x1,y1,x2,y2), če ga ne podamo obravnavam celo sliko
    def getColorFromImage(self, rgb_image, bounding_box=None, step=10):
        # print("heelo")

        h = rgb_image.shape[0]
        w = rgb_image.shape[1]

        if bounding_box == None:
            bounding_box = (0, 0, w, h)

        x1, y1, x2, y2 = bounding_box

        # cv2.imshow('image', rgb_image)  # prikazemo sliko
        # cv2.waitKey(0)

        colors = []

        # print("Calculating..")

        for y in range(y1, y2, int((y2 - y1) / step)):
            # for y in range(0, 5):
            for x in range(x1, x2, int((x2 - x1) / step)):
                b, g, r = rgb_image[y, x]
                colors.append(self.getColorNameFromRGB((r, g, b)))
                # print(y,x)
            # print("Processed row " + str(y))

        # print(colors)
        # print("Most common color: " + max(set(colors), key=colors.count))

        # vrnemo najpogostejso barvo
        return max(set(colors), key=colors.count)

    def importDataset(self, path):
        # print("Reading from " + path)

        with open(path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    # print(f'Column names are {", ".join(row)}')
                    a = "a"
                else:
                    self.data.append((row[0], row[1], row[2]))
                    self.labels.append(row[3])
                line_count += 1
            # print(f'Processed {line_count} lines.')

    # izpise confusion matrix in porocilo
    # test size pove kaksen del dataseta nej uporabi kot testno mnozico
    def evaluate(self, test_size=0.20):
        trainData, testData, trainLabels, testLabels = train_test_split(
            self.data, self.labels, test_size=0.20
        )
        self.model.fit(trainData, trainLabels)

        # print(predicted)

        print(confusion_matrix(testLabels, predicted))
        print(classification_report(testLabels, predicted))

        # fittamo nazaj na vse dane podatke, za druge metode
        self.model.fit(self.data, self.labels)

    def process_image_from_file(self, image_path):
        print("Processing new image: " + image_path)

        rgb_image = cv2.imread(image_path)  # preberemo sliko iz podane poti

        if rgb_image is None:
            print("Napaka pri branju slike!")
            return

        color = self.getColorFromImage(rgb_image)

        return color


class ColorDetectionNode:
    def __init__(self):
        self.cd = colorDetector("dataset.csv")
        self.bridge = CvBridge()

        # self.message_subscriber = rospy.Subscriber("/camera/depth/image_raw", color, self.message_callback)

        # rospy.init_node('add_two_ints_server') self.color_publisher = rospy.Publisher("color", Image, queue_size=10)
        rospy.init_node("color_detection_server")
        s = rospy.Service("color_detection", color, self.message_callback)

        print("Server running...")
        rospy.spin()

    def message_callback(self, data):
        print("Received new message!")

        image = self.bridge.imgmsg_to_cv2(data.image, desired_encoding="passthrough")
        bounding_box = data.bounding_box

        color = self.cd.getColorFromImage(image, bounding_box)

        # print(data.image)
        return color


if __name__ == "__main__":
    ColorDetectionNode()


"""
cd = colorDetector("dataset.csv")
rgb_image = cv2.imread("test.jpg") #preberemo sliko iz podane poti
bbox = (200,300,600,800)
color = cd.getColorFromImage(rgb_image, bbox)
print(color)
"""