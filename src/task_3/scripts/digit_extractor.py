import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pytesseract

from task_3.srv import ExtractDigitsService, ExtractDigitsServiceResponse


class DigitExtractor:
    def __init__(self):
        self.node = rospy.init_node("digit_extractor")
        self.srv = rospy.Service("digit_extractor", ExtractDigitsService, self.extract)

        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.adaptiveThreshConstant = 25
        self.aruco_params.adaptiveThreshWinSizeStep = 2

        self.image_publisher = rospy.Publisher("fine_navigation_image", Image, queue_size=10)
        rospy.spin()

        # r = rospy.Rate(1)
        # while not rospy.is_shutdown():
        #     self.extract(1)
        #     r.sleep()

    def extract(self, req):
        print("Start detection")
        try:
            self.image = self.bridge.imgmsg_to_cv2(
                rospy.wait_for_message("/camera/rgb/image_raw", Image), "bgr8"
            )
        except CvBridgeError as e:
            print(e)

        image = self.find_markers()
        if image is None:
            return ExtractDigitsServiceResponse(0, 1)
        res = self.extract_digits(image)
        print(res)
        return res

    def find_markers(self):
        img = self.image[:, : self.image.shape[1] // 2, :]
        img_aruco = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected_corners = cv2.aruco.detectMarkers(
            img_aruco, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is None:
            return

        image_size = (351, 248, 3)
        marker_side = 50

        img_out = np.zeros(image_size, np.uint8)
        img_out_corners = np.array(
            [
                [marker_side / 2, img_out.shape[0] - marker_side / 2],
                [img_out.shape[1] - marker_side / 2, img_out.shape[0] - marker_side / 2],
                [marker_side / 2, marker_side / 2],
                [img_out.shape[1] - marker_side / 2, marker_side / 2],
            ]
        )

        src_points = np.zeros((4, 2))
        marker_centers = np.zeros((4, 2))

        center_point = []
        for idx in ids:
            if len(corners) > idx[0] - 1:
                corners_centers = np.squeeze(corners[idx[0] - 1])
                marker_center = np.mean(corners_centers, axis=0)
                marker_centers[idx[0] - 1] = marker_center
                center_point = np.mean(marker_centers, axis=0)

        if len(center_point) == 0:
            return ExtractDigitsServiceResponse(0, 1)

        missing_markers = list()
        for i, e in enumerate(marker_centers):
            if e[0] == 0 and e[1] == 0:
                missing_markers.append(i)

        if missing_markers != []:
            max_x = max(marker_centers[:, 0])
            min_x = min([e for e in marker_centers[:, 0] if e != 0])
            max_y = max(marker_centers[:, 1])
            min_y = min([e for e in marker_centers[:, 1] if e != 0])

            if len(missing_markers) == 1:
                index = missing_markers[0]
                if not any(e[0] == min_x and e[1] == min_y for e in marker_centers):
                    marker_centers[index] = [min_x, min_y]
                elif not any(e[0] == min_x and e[1] == max_y for e in marker_centers):
                    marker_centers[index] = [min_x, max_y]
                elif not any(e[0] == max_x and e[1] == max_y for e in marker_centers):
                    marker_centers[index] = [max_x, max_y]
                elif not any(e[0] == max_x and e[1] == min_y for e in marker_centers):
                    marker_centers[index] = [max_x, min_y]

        if len(marker_centers) > 2:
            img_t = cv2.copyTo(self.image, mask=np.ones_like(self.image))
            for i, e in enumerate(marker_centers):
                img_t = cv2.rectangle(
                    img_t,
                    (int(e[0]) - 10, int(e[1]) - 10),
                    (int(e[0]) + 10, int(e[1]) + 10),
                    (255, 0, 0),
                    thickness=2,
                )
                img_t = cv2.putText(
                    img_t,
                    str(i),
                    (int(e[0]) - 10, int(e[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    2,
                    (0, 0, 255),
                )

            ros_img = self.bridge.cv2_to_imgmsg(img_t)
            self.image_publisher.publish(ros_img)

            for coords in marker_centers:
                #  Map the correct source points
                if coords[0] < center_point[0] and coords[1] < center_point[1]:
                    src_points[2] = coords
                elif coords[0] < center_point[0] and coords[1] > center_point[1]:
                    src_points[0] = coords
                elif coords[0] > center_point[0] and coords[1] < center_point[1]:
                    src_points[3] = coords
                else:
                    src_points[1] = coords

            h, status = cv2.findHomography(src_points, img_out_corners)
            img_out = cv2.warpPerspective(img_aruco, h, (img_out.shape[1], img_out.shape[0]))
            return img_out

    def extract_digits(self, image):
        img_out = image[125:221, 50:195]
        # Option 1 - use ordinairy threshold the image to get a black and white image
        # ret,img_out = cv2.threshold(img_out,100,255,0)

        # Option 1 - use adaptive thresholding
        # img_out = cv2.adaptiveThreshold(
        #     img_out, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5
        # )

        # Use Otsu's thresholding
        ret, img_out = cv2.threshold(img_out, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Pass some options to tesseract
        config = "--psm 13 outputbase nobatch digits"

        text = pytesseract.image_to_string(img_out, config=config)
        text = text.strip()
        if len(text) == 2:
            x = int(text[0])
            y = int(text[1])
            print("Found: ", x * 10 + y)
            return ExtractDigitsServiceResponse(x * 10 + y, 0)
        else:
            print("No digit found!")
            return ExtractDigitsServiceResponse(0, 1)


if __name__ == "__main__":
    DigitExtractor()