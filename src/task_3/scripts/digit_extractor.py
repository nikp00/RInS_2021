import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pytesseract
from time import sleep
import re
from google.cloud import vision
import io
import os
import sys
from task_3.srv import ExtractDigitsService, ExtractDigitsServiceResponse


class DigitExtractor:
    def __init__(self):
        self.node = rospy.init_node("digit_extractor")
        self.srv = rospy.Service("digit_extractor", ExtractDigitsService, self.extract)

        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.adaptiveThreshConstant = 10
        self.aruco_params.adaptiveThreshWinSizeStep = 2
        self.aruco_params.minCornerDistanceRate = 0.2
        self.aruco_params.polygonalApproxAccuracyRate = 0.07

        self.image_publisher = rospy.Publisher("fine_navigation_image", Image, queue_size=10)
        self.image_publisher123 = rospy.Publisher("fine_navigation_image_out", Image, queue_size=10)
        self.client = vision.ImageAnnotatorClient()

        rospy.spin()

    def extract(self, req):
        try:
            print("START")
            cv_image = self.bridge.imgmsg_to_cv2(
                rospy.wait_for_message("/camera/rgb/image_raw", Image), "bgr8"
            )

            height, width, channels = cv_image.shape

            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)

            #BLURING THE IMAGE
            blur = cv2.GaussianBlur(cv_image, (0, 0), 6)
            cv_image = cv2.addWeighted(cv_image, 1.5, blur, -0.5, 0)

            #TRESHOLDING
            ret,cv_image = cv2.threshold(cv_image,15,190,cv2.THRESH_BINARY)


            cropped_image = cv_image.copy()

            #ARUCO MARKER DETECTION
            corners, ids, rejected_corners = cv2.aruco.detectMarkers(cropped_image,self.aruco_dict,parameters=self.aruco_params)

            biggest_diff = 0
            prev_point = None

            all_corners = []

            # Save firsf number you found
            bestResult = None
            cvCopy = cv_image.copy()

            #Preveri vse positive cornerje
            if not corners is None:

                if len(corners) >= 2:
                    for idx in corners:
                        for cords in idx:
                            x111 = int(cords[0][0])
                            y222 = int(cords[0][1])

                            x1y1 = (x111, y222)
                            x2y2 = (x111+25, y222+25)

                            all_corners.append(x1y1)

                            #IZRISE KVADRAT OKOLI MARKERJA
                            cv_image = cv2.rectangle(cropped_image,x1y1, x2y2, (190,190,190), 25)
                            cvCopy = cv2.rectangle(cvCopy,x1y1, x2y2, (0,190,0), 5)

            # Preveri vse rejectane kornerje
            # if not rejected_corners is None:
            #
            #     if len(rejected_corners) >= 1:
            #         for idx in rejected_corners:
            #
            #             for cords in idx:
            #                 x111 = int(cords[0][0])
            #                 y222 = int(cords[0][1])
            #
            #                 x1y1 = (x111, y222)
            #                 x2y2 = (x111+20, y222+20)
            #
            #                 all_corners.append(x1y1)
            #
            #                 #IZRISE KVADRAT OKOLI MARKERJA
            #                 cv_image = cv2.rectangle(cropped_image,x1y1, x2y2, (190,190,190), 25)
            #                 cvCopy = cv2.rectangle(cvCopy,x1y1, x2y2, (0,190,0), 5)


            # cv2.imshow('Warped image',cvCopy)
            # cv2.waitKey(1)

            ros_img = self.bridge.cv2_to_imgmsg(cvCopy)
            self.image_publisher.publish(ros_img)
            combine = {}
            if len(all_corners) >= 3:
                for cords in all_corners:
                    for test in combine:
                        if abs(test-cords[1]) <= 15:
                            tmp = combine[test].copy()
                            new_key = int((test + cords[1])/2)
                            del combine[test]
                            combine[new_key] = tmp
                            combine[new_key].append(cords)
                            break
                    else:
                        combine[cords[1]] = [cords]


            max1 = None
            keyMax1 = None
            max2 = None
            keyMax2 = None

            for test in combine:
                if max1 == None:
                    max1 = len(combine[test])
                    keyMax1 = test
                elif max2 == None:
                    max2 = len(combine[test])
                    keyMax2 = test
                else:
                    if len(combine[test]) > max1:
                        max1 = len(combine[test])
                        keyMax1 = test
                    elif len(combine[test]) > max2:
                        max2 = len(combine[test])
                        keyMax2 = test

            leftCorners = {}
            leftCorners[keyMax1] = combine[keyMax1].copy()
            leftCorners[keyMax2] = combine[keyMax2].copy()

            okayCorners = []

            for cords in leftCorners[keyMax2]:
                for value in leftCorners[keyMax1]:
                    if(abs(value[0]-cords[0]) <= 15):
                        break
                else:
                    leftCorners[keyMax1].append((cords[0], keyMax2))

            leftCorners[keyMax1] = sorted(leftCorners[keyMax1])
            leftCorners[keyMax2] = sorted(leftCorners[keyMax2])

            #Iskanje primernih aruco markerjev -> Gremo cez dalsi array
            for i in range(0,len(leftCorners[keyMax1])-1):

                x1 = leftCorners[keyMax1][i][0]
                x2 = leftCorners[keyMax1][i+1][0]

                x1y1 = (x1, int(keyMax2))
                x2y2 = (x2, int(keyMax1))

                okayCorners.append((x1y1,x2y2))

                #Za prikaz odseka slik
                # color = list(np.random.choice(range(256), size=3))
                # test_image = cropped_image.copy()
                # test_image = cv2.rectangle(test_image,x1y1, x2y2, (int(color[0]),int(color[1]),int(color[2])), 5)
                # cv2.imshow('Warped image',test_image)
                # cv2.waitKey(10)
                # sleep(1)


            # Increase proportionally if you want a larger image
            image_size=(351,248,3)
            marker_side=50

            img_out = np.zeros(image_size, np.uint8)
            out_pts = np.array([[marker_side/2,img_out.shape[0]-marker_side/2],
                            [img_out.shape[1]-marker_side/2,img_out.shape[0]-marker_side/2],
                            [marker_side/2,marker_side/2],
                            [img_out.shape[1]-marker_side/2,marker_side/2]])

            src_points = np.zeros((4,2))
            cens_mars = np.zeros((4,2))


            for cords in okayCorners:

                img_out = np.zeros(image_size, np.uint8)
                out_pts = np.array([[marker_side/2,img_out.shape[0]-marker_side/2],
                                [img_out.shape[1]-marker_side/2,img_out.shape[0]-marker_side/2],
                                [marker_side/2,marker_side/2],
                                [img_out.shape[1]-marker_side/2,marker_side/2]])

                src_points = np.zeros((4,2))
                x1 = None
                x2 = None
                y1 = None
                y2 = None

                # Poskrbimo da so v pravilni orentaciji
                # 3        4
                # 1        2

                if cords[0][0] < cords[1][0]:
                    x1 = cords[0][0]
                    x2 = cords[1][0]
                else:
                    x1 = cords[1][0]
                    x2 = cords[0][0]

                if cords[0][1] < cords[1][1]:
                    y1 = cords[0][1]
                    y2 = cords[1][1]
                else:
                    y1 = cords[1][1]
                    y2 = cords[0][1]

                src_points[0] = (x1,y2)
                src_points[1] = (x2,y2)
                src_points[2] = (x1,y1)
                src_points[3] = (x2,y1)

                h, status = cv2.findHomography(src_points, out_pts)
                img_out = cv2.warpPerspective(cv_image, h, (img_out.shape[1],img_out.shape[0]))

                ################################################
                ####### Extraction of digits starts here #######
                ################################################

                # Cut out everything but the numbers
                img_out = img_out[90:290,40:300]

                # Option 1 - use ordinairy threshold the image to get a black and white image

                ret,img_out = cv2.threshold(img_out,100,255,0)

                # Creating kernel
                kernel = np.ones((5, 5), np.uint8)

                # Using cv2.erode() method
                # img_out = cv2.erode(img_out, kernel)

                ros_img = self.bridge.cv2_to_imgmsg(img_out)
                self.image_publisher123.publish(ros_img)
                # cv2.imshow('Podobna',img_out)
                # cv2.waitKey(1)
                # sleep(1)

                ##################################################################################
                ###############################GOOGLE API RECOGNITION#############################
                ##################################################################################
                success, encoded_image = cv2.imencode('.jpg', img_out)
                content2 = encoded_image.tobytes()
                image_cv2 = vision.Image(content=content2)
                response =  self.client.text_detection(image=image_cv2)
                texts = response.text_annotations
                number = texts[0].description

                number = re.findall('\\d+', number)

                if(len(number) >= 2):
                    myNumber = number[0] + number[1]
                    print("RESULT IS: ", myNumber)
                    return ExtractDigitsServiceResponse(int(myNumber), 0)


                #################################################################################

            #     # Pass some options to tesseract
            #     config = '--psm 3 outputbase nobatch digits'
            #     config1 = '--psm 13 outputbase nobatch digits'
            #
            #     # Visualize the image we are passing to Tesseract
            #     # cv2.imshow('Warped image',img_out)
            #     # cv2.waitKey(1)
            #
            #     # Extract text from image
            #     text = pytesseract.image_to_string(img_out, config=config)
            #
            #     # Remove any whitespaces from the left and right
            #     text = text.strip()
            #
            #     # If the extracted text is of the right length
            #     if len(text)>=2:
            #         x=int(text[0])
            #         y=int(text[1])
            #         if(int(x) >= 1):
            #             bestResult = re.findall('\\d+', text)[0]
            #             if(int(bestResult) < 100):
            #                 print("RESULT IS ", bestResult)
            #                 return ExtractDigitsServiceResponse(int(bestResult), 0)
            #
            #
            #
            #     # Extract text from image
            #     text = pytesseract.image_to_string(img_out, config=config1)
            #
            #     # Remove any whitespaces from the left and right
            #     text = text.strip()
            #
            #     # If the extracted text is of the right length
            #     if len(text)>=2:
            #         x=int(text[0])
            #         y=int(text[1])
            #         if(int(x) >= 1):
            #             bestResult = re.findall('\\d+', text)[0]
            #             if(int(bestResult) <= 100):
            #                 print("RESULT IS ", bestResult)
            #                 return ExtractDigitsServiceResponse(int(bestResult), 0)
            #
            # return ExtractDigitsServiceResponse(0, 1)

            return ExtractDigitsServiceResponse(0, 1)
        except:
            return ExtractDigitsServiceResponse(0, 1)

if __name__ == "__main__":
    auth = sys.argv[1]
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = auth
    DigitExtractor()
