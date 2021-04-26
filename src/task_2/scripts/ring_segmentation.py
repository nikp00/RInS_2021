#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class RingSegmentation:
    def __init__(self):
        self.node = rospy.init_node("ring_segmentation", anonymous=True)
        self.bridge = CvBridge()

        self.dims = (0, 0, 0)
        self.ring_markers = MarkerArray()
        self.seq = 1

        self.cv_image = None
        self.depth_image = None
        self.new_image = False

        # self.image_sub = rospy.Subscriber(
        #     "/camera/rgb/image_raw", Image, self.image_callback
        # )

        self.depth_image_subscriber = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.depth_callback
        )

        self.ring_markers_publisher = rospy.Publisher(
            "ring_markers", MarkerArray, queue_size=10
        )

        self.image_publisher = rospy.Publisher("ring_image", Image, queue_size=10)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.run()

    def run(self):
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.find_rings()
            self.ring_markers_publisher.publish(self.ring_markers)
            r.sleep()

    def find_rings(self):
        if not self.new_image:
            return
        self.new_image = False

        self.dims = self.cv_image.shape

        # gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        img = cv2.equalizeHist(self.cv_image)

        # img = self.cv_image

        ret, thresh = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )

        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)

        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(
                    ((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2)
                )
                if dist < 5:
                    candidates.append((e1, e2))

        print("Processing is done! found", len(candidates), "candidates for rings")

        skip = True
        for c in candidates:
            e1 = c[0]
            e2 = c[1]

            size = (e1[1][0] + e1[1][1]) / 2
            center = (e1[0][1], e1[0][0])

            if center[0] > self.cv_image.shape[0] / 3:
                continue

            skip = False

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)

            cv2.ellipse(self.cv_image, e1, 200, 2)
            cv2.ellipse(self.cv_image, e2, 200, 2)
            cv2.circle(
                self.cv_image, (int(center[1]), int(center[0])), 3, (0, 255, 0), 3
            )

            x_min = x1 if x1 > 0 else 0
            x_max = x2 if x2 < self.cv_image.shape[0] else self.cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < self.cv_image.shape[1] else self.cv_image.shape[1]

            # depth_image = self.bridge.imgmsg_to_cv2(depth_image, "16UC1")

            cv2.rectangle(self.cv_image, (y_max, x_max), (y_min, x_min), (0, 0, 255), 2)

            padding = 0

            self.get_pose(
                e1,
                np.nanmean(
                    np.ma.masked_equal(
                        self.depth_image[
                            x_min - padding : x_max + padding,
                            y_min - padding : y_max + padding,
                        ],
                        0,
                    )
                ),
            )

            np.set_printoptions(
                threshold=sys.maxsize,
            )

            print(
                "Mean",
                float(
                    np.nanmean(
                        self.depth_image[
                            x_min - padding : x_max + padding,
                            y_min - padding : y_max + padding,
                        ],
                    )
                ),
                float(
                    np.nanmean(
                        np.ma.masked_equal(
                            self.depth_image[
                                x_min - padding : x_max + padding,
                                y_min - padding : y_max + padding,
                            ],
                            0,
                        )
                    )
                ),
                np.ma.masked_equal(
                    self.depth_image[
                        x_min - padding : x_max + padding,
                        y_min - padding : y_max + padding,
                    ],
                    0,
                ),
                self.depth_image[x_min:x_max, y_min:y_max],
            )
            print(
                "Center",
                float(self.depth_image[int(center[0]), int(center[1])]),
                center,
            )

        if not skip and len(candidates) > 0:
            ros_img = self.bridge.cv2_to_imgmsg(self.cv_image)
            self.image_publisher.publish(ros_img)

    def get_pose(self, e, dist):
        k_f = 525

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x, k_f)
        print(
            "X angle: ",
            angle_to_target * (180 / np.pi),
            ", Y angle: ",
            np.arctan2(elipse_y, k_f) * (180 / np.pi),
        )

        print(
            "Distance: ",
            dist,
            ", Fixed dist: ",
            np.cos(np.arctan2(elipse_y, k_f)) * dist,
        )

        # dist = np.cos(np.arctan2(elipse_y, k_f)) * dist

        x, y = dist * np.cos(angle_to_target), dist * np.sin(angle_to_target)

        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = rospy.Time(0)

        tf_ready = False

        while not tf_ready:
            try:
                point_world = self.tf_buf.transform(point_s, "map")
                tf_ready = True
            except Exception as e:
                print(e)

        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z

        skip = False
        for e in self.ring_markers.markers:
            if (
                np.sqrt(
                    np.power(pose.position.x - e.pose.position.x, 2)
                    + np.power(pose.position.y - e.pose.position.y, 2)
                )
                < 0.5
            ):
                e.pose.position.x = float((pose.position.x + e.pose.position.x) / 2)
                e.pose.position.y = float((pose.position.y + e.pose.position.y) / 2)
                e.color = ColorRGBA(0, 1, 1, 1)
                skip = True
                break

        if not skip:
            self.seq += 1
            marker = Marker()
            marker.header.stamp = point_world.header.stamp
            marker.header.frame_id = point_world.header.frame_id
            marker.pose = pose
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Duration.from_sec(10)
            marker.id = self.seq
            marker.scale = Vector3(0.1, 0.1, 0.1)
            marker.color = ColorRGBA(0, 1, 0, 1)
            self.ring_markers.markers.append(marker)

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            self.depth_image = rospy.wait_for_message("/camera/depth/image_raw", Image)
            self.depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, "16UC1")
        except Exception as e:
            print(e)

        self.new_image = True

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            depth_16u = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        image = depth_16u / 65536.0 * 255
        image = image / np.max(image) * 255
        self.cv_image = np.array(image, dtype=np.uint8)

        self.new_image = True

        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image))


if __name__ == "__main__":
    RingSegmentation()
