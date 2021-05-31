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
        rospy.spin()

    def extract(self, req):
        print("Start detection")
        try:
            image = self.bridge.imgmsg_to_cv2(
                rospy.wait_for_message("/camera/rgb/image_raw", Image), "bgr8"
            )
        except CvBridgeError as e:
            print(e)
        print("IMG OK")

        img = image[:, : image.shape[1] // 2, :]
        blurred = cv2.GaussianBlur(img, (0, 0), 6)
        img = cv2.addWeighted(img, 1.5, blurred, -0.5, 0)
        # cv2.imshow("test", img)
        # cv2.waitKey(1)

        print("Image recieved")

        corners, ids, rejected_corners = cv2.aruco.detectMarkers(
            img, self.aruco_dict, parameters=self.aruco_params
        )
        image_size = (351, 248, 3)
        marker_side = 50
        print("Aruco computed")

        img_out = np.zeros(image_size, np.uint8)
        out_pts = np.array(
            [
                [marker_side / 2, img_out.shape[0] - marker_side / 2],
                [img_out.shape[1] - marker_side / 2, img_out.shape[0] - marker_side / 2],
                [marker_side / 2, marker_side / 2],
                [img_out.shape[1] - marker_side / 2, marker_side / 2],
            ]
        )

        src_points = np.zeros((4, 2))
        cens_mars = np.zeros((4, 2))

        if not ids is None:
            print("IDs found")

            cen_point = []
            for idx in ids:
                if idx[0] - 1 >= len(corners):
                    continue
                cors = np.squeeze(corners[idx[0] - 1])
                cen_mar = np.mean(cors, axis=0)
                cens_mars[idx[0] - 1] = cen_mar
                cen_point = np.mean(cens_mars, axis=0)

            if len(cen_point) == 0:
                return ExtractDigitsServiceResponse(0, 1)

            for coords in cens_mars:
                #  Map the correct source points
                if coords[0] < cen_point[0] and coords[1] < cen_point[1]:
                    src_points[2] = coords
                elif coords[0] < cen_point[0] and coords[1] > cen_point[1]:
                    src_points[0] = coords
                elif coords[0] > cen_point[0] and coords[1] < cen_point[1]:
                    src_points[3] = coords
                else:
                    src_points[1] = coords

            print("Cords computed")
            h, status = cv2.findHomography(src_points, out_pts)
            img_out = cv2.warpPerspective(image, h, (img_out.shape[1], img_out.shape[0]))

            img_t = cv2.copyTo(image, mask=np.ones_like(image))
            for e in cens_mars:
                img_t = cv2.rectangle(
                    img_t,
                    (int(e[0]) - 10, int(e[1]) - 10),
                    (int(e[0]) + 10, int(e[1]) + 10),
                    (255, 0, 0),
                    thickness=2,
                )
            # cv2.imshow("2image", img_t)
            # cv2.waitKey(1)

            ################################################
            #### Extraction of digits starts here
            ################################################

            # Cut out everything but the numbers
            img_out = img_out[125:221, 50:195, :]

            # Convert the image to grayscale
            img_out = cv2.cvtColor(img_out, cv2.COLOR_BGR2GRAY)

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

            # Visualize the image we are passing to Tesseract
            # cv2.imshow("Warped image", img_out)
            # cv2.waitKey(1)

            text = pytesseract.image_to_string(img_out, config=config)
            text = text.strip()
            print("Teseract finished")

            if len(text) == 2:
                x = int(text[0])
                y = int(text[1])
                print("Found: ", x * 10 + y)
                return ExtractDigitsServiceResponse(x * 10 + y, 0)
            else:
                print("No digit found!")
                return ExtractDigitsServiceResponse(0, 1)
        print("No digit found!")
        return ExtractDigitsServiceResponse(0, 1)


if __name__ == "__main__":
    DigitExtractor()