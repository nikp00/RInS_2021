# pip3 install -U scikit-learn

from timeit import default_timer as timer

import csv
import cv2
import numpy as np
from sklearn import preprocessing
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix

import os
import pickle

# https://medium.com/analytics-vidhya/building-rgb-color-classifier-part-1-af58e3bcfef7
# https://github.com/AjinkyaChavan9/RGB-Color-Classifier-with-Deep-Learning-using-Keras-and-Tensorflow/blob/master/Dataset/final_data.csv


COLOR_DEFINITIONS = {
    "blue": (0, 0, 255),
    "black": (0, 0, 0),
    "grey": (128, 128, 128),
    "red": (255, 0, 0),
    "pink": (255, 192, 203),
    "green": (0, 255, 0),
    "white": (255, 255, 255),
    "purple": (128, 0, 128),
    "yellow": (255, 255, 0),
}


class ColorClassifier:

    # path = pot do dataseta, k = stevilo sosedov za knn (default = 3)
    def __init__(self, path, k=3):
        self.data = []
        self.labels = []

        self.importDataset(path)

        self.model = KNeighborsClassifier(k)
        self.model.fit(self.data, self.labels)

    def getColorNameFromRGB(self, RGBValue):
        return self.model.predict([list(RGBValue)])[0]

    # Bounding box je tuple (x1,y1,x2,y2), če ga ne podamo obravnavam celo sliko
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

    def importDataset(self, path):
        print("Reading from " + path)

        with open(path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    print(f'Column names are {", ".join(row)}')
                else:
                    self.data.append((row[0], row[1], row[2]))
                    self.labels.append(row[3].lower())
                line_count += 1
            print(f"Processed {line_count} lines.")

    # vrne tuple z rgb vrednostmi
    def hexToRgb(self, hexValue):
        hexValue.lstrip("#")
        return tuple(int(h[i : i + 2], 16) for i in (0, 2, 4))

    # izpise confusion matrix in porocilo
    # test size pove kaksen del dataseta nej uporabi kot testno mnozico
    def evaluate(self, test_size=0.20):
        trainData, testData, trainLabels, testLabels = train_test_split(
            self.data, self.labels, test_size=0.20
        )
        self.model.fit(trainData, trainLabels)

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

    def export_model(self, path):
        pickle.dump(self.model, open(path, "wb"))
        print(f"Model exported to: {path}")

    def export_labels(self, path):
        pickle.dump(COLOR_DEFINITIONS, open(path, "wb"))
        print()
        print(f"Labels exported to: {path}")


base_path = os.path.abspath(os.pardir)
dataset_path = base_path + "/data/color_classifier_data/dataset.csv"
test_img_path = base_path + "/data/color_classifier_data/test.jpg"
export_path = base_path + "/data/color_classifier_data/model.pickle"
labels_path = base_path + "/data/color_classifier_data/labels.pickle"

cd = ColorClassifier(dataset_path)

cd.export_model(export_path)
cd.export_labels(labels_path)

rgb_image = cv2.imread(test_img_path)  # preberemo sliko iz podane poti
bbox = (200, 300, 600, 800)
color = cd.getColorFromImage(rgb_image, bbox)
print(color)
