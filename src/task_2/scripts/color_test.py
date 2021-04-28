import sys
import rospy
from task_2.srv import color
from cv_bridge import CvBridge

import cv2

bridge = CvBridge()

def colorClient():
    rgb_image = cv2.imread("test.jpg")
    image_message = bridge.cv2_to_imgmsg(rgb_image, encoding="passthrough")
    bounding_box = (0,0, 100, 100)

    msg = color()
    msg.image = image_message
    msg.bounding_box = bounding_box

    rospy.wait_for_service('color_detection')
    try:
        cd = rospy.ServiceProxy('color_detection', color)
        response = cd(msg.image, bounding_box)

        print("Response: ", response.color)

        return response.color

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("Running...")
    colorClient()
    #print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))