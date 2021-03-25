#!/usr/bin/python3



"""
Nalaganje potrebnih paketov:

sudo apt update
sudo apt install python3-pip
pip3 install dlib


Zalaufaj v konzoli: python3 face_detector_test.py

Pritisni q za izhod

"""



import cv2
import numpy as np

class face_finder:
    def __init__(self):
        #dnn za zaznavanje obrazov
        #self.face_net = cv2.dnn.readNetFromCaffe('/home/urban/ROS/src/exercise4/scripts/deploy.prototxt.txt', '/home/urban/ROS/src/exercise4/scripts/res10_300x300_ssd_iter_140000.caffemodel')
        self.face_net = cv2.dnn.readNetFromCaffe('deploy.prototxt.txt', 'res10_300x300_ssd_iter_140000.caffemodel')

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

    #na podani sliki oznaci obraze
    def find_faces(self, rgb_image):
        if rgb_image is None:
            print("Napaka pri branju slike!")
            return

        cv2.imshow('image',rgb_image)   #prikazemo sliko

        # Set the dimensions of the image
        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]

        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))

        #zaznamo obraze
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()

        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            #print("Confidence: " + str(confidence))
            if confidence > 0.5:
                print("Found a face")
                box = face_detections[0,0,i,3:7] * np.array([w,h,w,h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]     #koordinate bounding rectangla obraza

                print("x1: ", x1, "y1: " ,y1, "x2: ", x2, "y2: ",y2)

                cv2.rectangle(rgb_image,(x1,y1),(x2,y2),(0,255,0),3) #narisemo pravokotnik okoli obraza
                cv2.imshow('image',rgb_image)

        #cv2.waitKey(0)

    #odpre sliko iz podane poti in an njej oznaci obraze
    def process_image_from_file(self, image_path):
        print('Processing new image: ' + image_path)

        rgb_image = cv2.imread(image_path) #preberemo sliko iz podane poti

        if rgb_image is None:
            print("Napaka pri branju slike!")
            return

        self.find_faces(rgb_image)

        cv2.waitKey(0)


    #zajame video iz laptop kamere in na njem v zivo oznacuje obraze (pritisni q ali ctrl+c za ustavitev zajemanja)
    def display_from_camera(self):
        cap = cv2.VideoCapture(0)

        while(True):
            # Capture frame-by-frame
            ret, rgb_image = cap.read()

            rgb_image = cv2.flip(rgb_image, 1) # zrcalimo preko y osi

        
            self.find_faces(rgb_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()


def main():
        finder = face_finder()

        #inder.process_image_from_file("test.jpg")
        finder.display_from_camera()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()