#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from collections import defaultdict
import copy


from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from task_2.msg import PoseAndColor, PoseAndColorArray

from task_2.srv import ColorClassifierService, ColorClassifierServiceRequest


class RingSegmentation:
    def __init__(self):
        self.node = rospy.init_node("ring_segmentation", anonymous=True)

        self.bridge = CvBridge()
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.dims = (0, 0, 0)
        self.rings = list()
        self.seq = 1

        self.cv_image = None
        self.depth_image = None
        self.new_image = False

        self.params = {
            "invert_image": rospy.get_param("~invert_image", default=True),
            "equalize_hist": rospy.get_param("~equalize_hist", default=True),
        }

        self.ring_markers_publisher = rospy.Publisher(
            "ring_markers", MarkerArray, queue_size=10
        )
        self.n_detections_marker_publisher = rospy.Publisher(
            "ring_n_detections_markers", MarkerArray, queue_size=10
        )
        self.ring_pose_publisher = rospy.Publisher(
            "ring_pose", PoseAndColorArray, queue_size=10
        )
        self.image_publisher = rospy.Publisher("ring_image", Image, queue_size=10)

        self.get_color = rospy.ServiceProxy("/color_classifier", ColorClassifierService)
        rospy.wait_for_service("/color_classifier")

        self.depth_image_subscriber = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.depth_callback
        )

        self.run()

    def run(self):
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.find_rings()
            r.sleep()

    def find_rings(self):
        if not self.new_image:
            return

        self.new_image = False
        self.dims = self.cv_image.shape

        if self.params["invert_image"]:
            temp = np.copy(self.depth_image)
            temp[np.isnan(temp)] = 0
            mask = cv2.inRange(temp, 0, 0)
            self.cv_image = cv2.bitwise_or(self.cv_image, mask)

        if self.params["equalize_hist"]:
            self.cv_image = cv2.equalizeHist(self.cv_image)

        ret, thresh = cv2.threshold(self.cv_image, 50, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )

        elps = []
        for cnt in contours:
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

            cv2.rectangle(self.cv_image, (y_max, x_max), (y_min, x_min), (0, 0, 255), 2)

            padding = 0
            pose = self.get_pose(
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

            try:
                image = self.bridge.imgmsg_to_cv2(
                    rospy.wait_for_message("/camera/rgb/image_raw", Image), "bgr8"
                )
            except CvBridgeError as e:
                print(e)

            m1 = np.zeros(image.shape[0:2], np.uint8)
            m2 = np.zeros(image.shape[0:2], np.uint8)

            cv2.ellipse(m1, e1, (255, 255, 255), -1)
            cv2.ellipse(m2, e2, (255, 255, 255), -1)
            m3 = m2 - m1

            bgr_color = cv2.mean(image, mask=m3)
            print("bgr", bgr_color)

            req = ColorClassifierServiceRequest()
            req.mode = 1
            req.color = ColorRGBA(*[int(e) for e in bgr_color[::-1][1:]], 1)
            res = self.get_color(req)

            self.add_ring(pose, res)

        ros_img = self.bridge.cv2_to_imgmsg(self.cv_image)
        self.image_publisher.publish(ros_img)

    def get_pose(self, e, dist) -> Pose:
        k_f = 525

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x, k_f)
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

        return pose

    def add_ring(self, pose: Pose, res):
        skip = False
        for e in self.rings:
            if (
                np.sqrt(
                    np.power(pose.position.x - e.pose.position.x, 2)
                    + np.power(pose.position.y - e.pose.position.y, 2)
                )
                < 0.5
            ):
                e.pose.position.x = float((pose.position.x + e.pose.position.x) / 2)
                e.pose.position.y = float((pose.position.y + e.pose.position.y) / 2)

                e.color[
                    (res.marker_color.r, res.marker_color.g, res.marker_color.b)
                ] += 1

                e.n_detections += 1

                skip = True
                break

        if not skip:
            ring = Ring(
                pose,
                (res.marker_color.r, res.marker_color.g, res.marker_color.b),
                res.color,
                self.seq,
            )
            self.seq += 1
            self.rings.append(ring)
            print("New ring", res.marker_color, res.color)

        self.ring_markers_publisher.publish([e.to_marker() for e in self.rings])
        self.n_detections_marker_publisher.publish([e.to_text() for e in self.rings])
        self.ring_pose_publisher.publish(
            PoseAndColorArray([PoseAndColor(e.pose, e.color_name) for e in self.rings])
        )

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

        # self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image))


class Ring:
    def __init__(self, pose: Pose, color: tuple, color_name: str, id: int):
        self.pose = pose
        self.color = defaultdict(int)
        self.color[color] += 1
        self.id = id
        self.n_detections = 1
        self.color_name = color_name

    def to_marker(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()

        m.id = self.id
        m.ns = "ring_marker"
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = copy.deepcopy(self.pose)
        m.pose.position.z = 0.2
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        # m.color = ColorRGBA(*self.color, 1)
        color = max(self.color, key=self.color.get)
        color = map(lambda x: x / 255, color)
        m.color = ColorRGBA(*color, 1)

        m.lifetime = rospy.Duration(0)
        return m

    def to_text(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()

        m.id = self.id
        m.ns = "ring_n_detections_markers"
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose = copy.deepcopy(self.pose)
        m.pose.position.z = 1
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        m.color = ColorRGBA(0, 0, 0, 1)
        m.lifetime = rospy.Duration(0)

        m.text = str(self.n_detections)
        return m


if __name__ == "__main__":
    RingSegmentation()
