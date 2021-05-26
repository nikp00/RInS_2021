#!/usr/bin/python3

import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import tf
from collections import defaultdict
import copy


from sensor_msgs.msg import Image
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    Vector3,
    Pose,
    Point,
    Quaternion,
)
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from task_2.msg import PoseAndColor, PoseAndColorArray
from nav_msgs.msg import OccupancyGrid


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

        self.sent_ids = list()

        self.params = {
            "invert_image": rospy.get_param("~invert_image", default=False),
            "equalize_hist": rospy.get_param("~equalize_hist", default=False),
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

        rospy.wait_for_service("/color_classifier")
        self.get_color = rospy.ServiceProxy("/color_classifier", ColorClassifierService)

        self.map_msg = rospy.wait_for_message("/map", OccupancyGrid)

        self.depth_image_subscriber = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.depth_callback
        )

        self.occupancy_grid_to_img()
        self.run()

    def run(self):
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.find_rings()
            r.sleep()

    def occupancy_grid_to_img(self):
        self.map = np.flip(
            np.array([[e, e, e] for e in self.map_msg.data], dtype=np.uint8).reshape(
                (self.map_msg.info.height, self.map_msg.info.width, 3)
            ),
            axis=0,
        )

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

            try:
                image_msg = rospy.wait_for_message("/camera/rgb/image_raw", Image)
                image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError as e:
                print(e)

            m1 = np.zeros(image.shape[0:2], np.uint8)
            m2 = np.zeros(image.shape[0:2], np.uint8)

            cv2.ellipse(m1, e1, (255, 255, 255), -1)
            cv2.ellipse(m2, e2, (255, 255, 255), -1)
            m3 = m2 - m1

            padding = 0
            np.set_printoptions(threshold=np.inf, precision=2)

            temp = cv2.bitwise_and(self.depth_image, self.depth_image, mask=m3)
            temp[temp == 0] = np.nan
            pose = self.get_pose(
                e1,
                np.nanmean(
                    temp
                    # np.ma.masked_equal(
                    #     self.depth_image[
                    #         x_min - padding : x_max + padding,
                    #         y_min - padding : y_max + padding,
                    #     ],
                    #     0,
                    # )
                ),
                image_msg.header.stamp,
            )

            bgr_color = cv2.mean(image, mask=m3)

            req = ColorClassifierServiceRequest()
            req.mode = 1
            req.color = ColorRGBA(*[int(e) for e in bgr_color[::-1][1:]], 1)
            res = self.get_color(req)

            self.add_ring(pose, res)

        ros_img = self.bridge.cv2_to_imgmsg(self.cv_image)
        self.image_publisher.publish(ros_img)

    def get_pose(self, e, dist, stamp) -> Pose:
        k_f = 525

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x, k_f)

        # dist = np.cos(np.arctan2(elipse_y, k_f)) * dist
        # dist = np.sqrt(dist ** 2 - 0.71 ** 2)

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

        base_pose = self.get_current_pose(stamp)
        angle = tf.transformations.euler_from_quaternion(
            [
                base_pose.pose.orientation.x,
                base_pose.pose.orientation.y,
                base_pose.pose.orientation.z,
                base_pose.pose.orientation.w,
            ]
        )[2]

        angle = np.arctan2(
            base_pose.pose.position.y - pose.position.y,
            base_pose.pose.position.x - pose.position.x,
        )

        pose.orientation = Quaternion(
            *list(tf.transformations.quaternion_from_euler(0, 0, angle))
        )

        return pose

    def get_current_pose(self, time):
        pose_translation = None
        while pose_translation is None:
            try:
                pose_translation = self.tf_buf.lookup_transform(
                    "map", "base_link", time, rospy.Duration(2)
                )
            except Exception as e:
                print(e)

        pose = PoseStamped()
        pose.header.seq = 0
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(
            pose_translation.transform.translation.x,
            pose_translation.transform.translation.y,
            0,
        )
        pose.pose.orientation = pose_translation.transform.rotation
        return pose

    def add_ring(self, pose: Pose, res):
        skip = False
        for e in self.rings:
            if (
                np.sqrt(
                    np.power(pose.position.x - e.pose.position.x, 2)
                    + np.power(pose.position.y - e.pose.position.y, 2)
                )
                < 1
            ):
                # e.pose.position.x = float((pose.position.x + e.pose.position.x) / 2)
                # e.pose.position.y = float((pose.position.y + e.pose.position.y) / 2)
                e.x.append(pose.position.x)
                e.y.append(pose.position.y)
                e.calculate_pose()
                e.pose.orientation = pose.orientation
                e.color_name[res.color] += 1
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

        self.ring_markers_publisher.publish([e.to_marker() for e in self.rings])
        self.n_detections_marker_publisher.publish([e.to_text() for e in self.rings])

        poses = list()

        for e in self.sent_ids:
            for x in self.rings:
                if x.id == e:
                    poses.append(PoseAndColor(x.to_pose(), x.get_color_name(), e.id))

        for e in self.rings:
            if e.id not in self.sent_ids and e.n_detections > 1:
                poses.append(PoseAndColor(e.to_pose(), e.get_color_name(), e.id))
                self.sent_ids.append(e.id)

        self.ring_pose_publisher.publish(PoseAndColorArray(poses))

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


class Ring:
    def __init__(self, pose: Pose, color: tuple, color_name: str, id: int):
        self.pose = pose
        self.x = [pose.position.x]
        self.y = [pose.position.y]
        self.color = defaultdict(int)
        self.color[color] += 1
        self.id = id
        self.n_detections = 1
        self.color_name = defaultdict(int)
        self.color_name[color_name] += 1
        # self.pose.orientation = Quaternion(0, 0, 0, 1)

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
        color = max(self.color, key=self.color.get)
        color = map(lambda x: x / 255, color)
        if self.n_detections > 1:
            m.color = ColorRGBA(*color, 1)
        else:
            m.color = ColorRGBA(*color, 0.3)

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
        if self.n_detections > 1:
            m.color = ColorRGBA(0, 0, 0, 1)
        else:
            m.color = ColorRGBA(255, 0, 0, 1)
        m.lifetime = rospy.Duration(0)

        m.text = str(self.n_detections)
        return m

    def to_pose(self):
        pose = copy.deepcopy(self.pose)
        pose.position.z = 0
        return pose

    def get_color_name(self):
        return max(self.color_name, key=self.color_name.get)

    def calculate_pose(self):
        self.pose.position.x = np.mean(self.x)
        self.pose.position.y = np.mean(self.y)


if __name__ == "__main__":
    RingSegmentation()
