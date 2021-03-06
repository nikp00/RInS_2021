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
import math

from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import Image
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    TransformStamped,
    Vector3,
    Pose,
    Point,
    Quaternion,
)
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from task_3.msg import PoseAndColor, PoseAndColorArray
from nav_msgs.msg import OccupancyGrid


from task_3.srv import ColorClassifierService, ColorClassifierServiceRequest


class RingSegmentation:
    def __init__(self):
        self.node = rospy.init_node("ring_segmentation", anonymous=True)

        self.MIN_DETECTIONS = rospy.get_param("~min_detections", default=1)

        self.bridge = CvBridge()
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.dims = (0, 0, 0)
        self.rings = list()
        self.seq = 1

        self.cv_image = None
        self.depth_image = None
        self.new_image = False

        self.map = None

        self.map_msg = rospy.wait_for_message("/map", OccupancyGrid)
        self.map_transform = TransformStamped()
        self.map_transform.transform.translation.x = self.map_msg.info.origin.position.x
        self.map_transform.transform.translation.y = self.map_msg.info.origin.position.y
        self.map_transform.transform.translation.z = self.map_msg.info.origin.position.z
        self.map_transform.transform.rotation = self.map_msg.info.origin.orientation

        self.params = {
            "invert_image": rospy.get_param("~invert_image", default=False),
            "equalize_hist": rospy.get_param("~equalize_hist", default=False),
        }

        self.ring_markers_publisher = rospy.Publisher("ring_markers", MarkerArray, queue_size=10)
        self.n_detections_marker_publisher = rospy.Publisher(
            "ring_n_detections_markers", MarkerArray, queue_size=10
        )
        self.ring_pose_publisher = rospy.Publisher("ring_pose", PoseAndColorArray, queue_size=10)
        self.image_publisher = rospy.Publisher("ring_image", Image, queue_size=10)

        rospy.wait_for_service("/color_classifier")
        self.get_color = rospy.ServiceProxy("/color_classifier", ColorClassifierService)

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

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

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
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
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
            cv2.circle(self.cv_image, (int(center[1]), int(center[0])), 3, (0, 255, 0), 3)

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

            np.set_printoptions(threshold=np.inf, precision=2)

            temp = cv2.bitwise_and(self.depth_image, self.depth_image, mask=m3)
            temp[temp == 0] = np.nan
            pose = self.get_pose(
                e1,
                np.nanmean(temp),
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
        angle_to_target = np.arctan2(elipse_x, k_f)

        dist_from_center = abs(self.dims[1] / 2 - e[0][0])
        dist_compensation = 0.2 / (self.dims[1] / 2)

        print("Dims", self.dims[1])
        print("Dist form cernter", dist_from_center)
        print("Compensation", dist_from_center * dist_compensation)
        dist += dist_compensation * dist_from_center

        # dist = math.sqrt(math.pow(dist, 2) - math.pow(0.71, 2))
        x, y = dist * np.cos(angle_to_target), dist * np.sin(angle_to_target)
        if not stamp:
            stamp = rospy.Time.now()
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

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

        x = pose.position.x
        y = pose.position.y

        if np.isnan(x) or np.isnan(y):
            print("Nan")
            return pose

        grid_x = int((x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        grid_y = self.map_msg.info.height - int(
            (y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution
        )
        cell = self.map[grid_y, grid_x]
        target = 0 if cell[0] == 100 else 100
        left = self.check_direction(grid_x, grid_y, dx=-1, target=target)
        top = self.check_direction(grid_x, grid_y, dy=1, target=target)
        right = self.check_direction(grid_x, grid_y, dx=1, target=target)
        bottom = self.check_direction(grid_x, grid_y, dy=-1, target=target)
        dists = [e for e in [left, top, right, bottom] if e[2] != 0]
        min_pose = min(dists, key=lambda x: x[2])

        print("Ring target", target)
        grid_x = min_pose[0]
        grid_y = min_pose[1]
        print("Before", grid_x, grid_y)
        grid_x, grid_y = self.fix_distance_from_wall(grid_x, grid_y, min_distance=1)
        print("After", grid_x, grid_y)
        print()
        print()
        grid_y = (self.map_msg.info.height - grid_y) * self.map_msg.info.resolution
        grid_x = grid_x * self.map_msg.info.resolution
        pt = PointStamped()
        pt.point.x = grid_x
        pt.point.y = grid_y
        pt.point.z = 0
        pt = do_transform_point(pt, self.map_transform)

        pose = Pose()
        pose.position = pt.point

        return pose

    def fix_distance_from_wall(self, grid_x, grid_y, min_distance=6):
        if self.map[int(grid_y), int(grid_x)][0] == 0:
            done = False
            grid_x = int(grid_x)
            grid_y = int(grid_y)
            while not done:
                done = True
                left = self.check_direction(grid_x, grid_y, dx=-1, target=100)
                top = self.check_direction(grid_x, grid_y, dy=1, target=100)
                right = self.check_direction(grid_x, grid_y, dx=1, target=100)
                bottom = self.check_direction(grid_x, grid_y, dy=-1, target=100)
                if left[2] < min_distance:
                    grid_x += 1
                    done = False
                if right[2] < min_distance:
                    grid_x -= 1
                    done = False
                if bottom[2] < min_distance:
                    grid_y += 1
                    done = False
                if top[2] < min_distance:
                    grid_y -= 1
                    done = False
        return grid_x, grid_y

    def calculate_navigation_pose(self, x, y):
        if np.isnan(x) or np.isnan(y):
            return None
        grid_x = int((x - self.map_msg.info.origin.position.x) / self.map_msg.info.resolution)
        grid_y = self.map_msg.info.height - int(
            (y - self.map_msg.info.origin.position.y) / self.map_msg.info.resolution
        )
        cell = self.map[grid_y, grid_x]
        target = 0 if cell[0] == 100 else 100

        left = self.check_direction(grid_x, grid_y, dx=-1, target=target)
        top = self.check_direction(grid_x, grid_y, dy=1, target=target)
        right = self.check_direction(grid_x, grid_y, dx=1, target=target)
        bottom = self.check_direction(grid_x, grid_y, dy=-1, target=target)

        dists = [e for e in [left, top, right, bottom] if e[2] != 0]
        if len(dists) > 0:
            new_x, new_y, dist = min(dists, key=lambda x: x[2])
            if target == 0:
                direction = (new_x - grid_x, new_y - grid_y)
            else:
                direction = (grid_x - new_x, grid_y - new_y)

            magnitude = np.sqrt(np.power(direction[0], 2) + np.power(direction[1], 2))

            dx = direction[0] / magnitude
            dy = direction[1] / magnitude

            grid_x = grid_x + dx * 0.05 * 50
            grid_y = grid_y + dy * 0.05 * 50

            grid_x, grid_y = self.fix_distance_from_wall(grid_x, grid_y, 5)

            grid_y = (self.map_msg.info.height - grid_y) * self.map_msg.info.resolution
            grid_x = grid_x * self.map_msg.info.resolution
            pt = PointStamped()
            pt.point.x = grid_x
            pt.point.y = grid_y
            pt.point.z = 0
            pt = do_transform_point(pt, self.map_transform)

            pose = Pose()
            pose.position = pt.point

            angle = np.arctan2(
                y - pose.position.y,
                x - pose.position.x,
            )

            if angle < 0:
                angle += 2 * math.pi

            pose.orientation = Quaternion(
                *list(tf.transformations.quaternion_from_euler(0, 0, angle))
            )
            return pose
        return None

    def check_direction(self, x, y, dx=0, dy=0, target=0):
        start_x = x
        start_y = y
        while (
            (0 < x < self.map.shape[1])
            and (0 < y < self.map.shape[0])
            and self.map[y, x][0] != target
        ):
            y += dy
            x += dx

        dist = np.sqrt(np.power(x - start_x, 2) + np.power(y - start_y, 2))
        return x, y, dist

    def add_ring(self, pose: Pose, res):
        navigation_pose = self.calculate_navigation_pose(pose.position.x, pose.position.y)
        if navigation_pose == None:
            return
        skip = False
        for e in self.rings:
            if (
                np.sqrt(
                    np.power(pose.position.x - e.obj_pose.position.x, 2)
                    + np.power(pose.position.y - e.obj_pose.position.y, 2)
                )
                < 0.4
                and res.color == e.get_color_name()
            ):

                e.add_pose(pose, navigation_pose)
                e.color_name[res.color] += 1
                e.color[(res.marker_color.r, res.marker_color.g, res.marker_color.b)] += 1
                e.n_detections += 1

                skip = True
                break

        if not skip:
            ring = Ring(
                pose,
                navigation_pose,
                (res.marker_color.r, res.marker_color.g, res.marker_color.b),
                res.color,
                self.seq,
                self.MIN_DETECTIONS,
            )
            self.seq += 1
            self.rings.append(ring)

        count = {"red": 0, "green": 0, "black": 0, "blue": 0}
        filtered_rings = {"red": None, "green": None, "black": None, "blue": None}

        for e in self.rings:
            if count[e.get_color_name()] < e.n_detections:
                count[e.get_color_name()] = e.n_detections
                filtered_rings[e.get_color_name()] = e

        filtered_rings = [v for k, v in filtered_rings.items() if v != None]

        self.ring_markers_publisher.publish(
            [e.to_marker() for e in filtered_rings]
            + [e.to_navigation_marker() for e in filtered_rings]
        )
        self.n_detections_marker_publisher.publish([e.to_text() for e in filtered_rings])

        self.ring_pose_publisher.publish(
            PoseAndColorArray(
                [
                    PoseAndColor(e.obj_pose, e.navigation_pose, e.get_color_name(), e.id)
                    for e in filtered_rings
                    if e.n_detections > self.MIN_DETECTIONS
                ]
            )
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


class Ring:
    def __init__(
        self,
        obj_pose: Pose,
        navigation_pose: Pose,
        color: tuple,
        color_name: str,
        id: int,
        min_detections,
    ):
        self.obj_pose = obj_pose
        self.navigation_pose = navigation_pose
        self.ox = [obj_pose.position.x]
        self.oy = [obj_pose.position.y]
        self.nx = [navigation_pose.position.x]
        self.ny = [navigation_pose.position.y]
        self.na = [self.quaternion_to_euler(navigation_pose)]
        self.color = defaultdict(int)
        self.color[color] += 1
        self.id = id
        self.n_detections = 1
        self.color_name = defaultdict(int)
        self.color_name[color_name] += 1
        self.min_detections = min_detections

    def add_pose(self, obj_pose: Pose, navigation_pose: Pose):
        navigation_pose.position.z = 0
        self.ox.append(obj_pose.position.x)
        self.oy.append(obj_pose.position.y)
        self.nx.append(navigation_pose.position.x)
        self.ny.append(navigation_pose.position.y)
        self.na.append(self.quaternion_to_euler(navigation_pose))
        self.calculate_pose()
        self.calculate_rotations()

    def to_marker(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.id = self.id
        m.ns = "ring_marker"
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = copy.deepcopy(self.obj_pose)
        m.pose.position.z = 0.2
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        color = max(self.color, key=self.color.get)
        color = map(lambda x: x / 255, color)
        if self.n_detections > self.min_detections:
            m.color = ColorRGBA(*color, 1)
        else:
            m.color = ColorRGBA(*color, 0.3)

        m.lifetime = rospy.Duration(0)
        return m

    def to_navigation_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration(0)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.pose = self.navigation_pose
        color = max(self.color, key=self.color.get)
        color = map(lambda x: x / 255, color)
        if self.n_detections > self.min_detections:
            marker.color = ColorRGBA(*color, 1)
        else:
            marker.color = ColorRGBA(*color, 0.3)
        marker.id = self.id + 100
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0
        return marker

    def to_text(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.id = self.id
        m.ns = "ring_n_detections_markers"
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose = copy.deepcopy(self.obj_pose)
        m.pose.position.z = 1
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        if self.n_detections > self.min_detections:
            m.color = ColorRGBA(0, 0, 0, 1)
        else:
            m.color = ColorRGBA(255, 0, 0, 1)
        m.lifetime = rospy.Duration(0)

        m.text = str(self.n_detections)
        return m

    def get_color_name(self):
        return max(self.color_name, key=self.color_name.get)

    def calculate_pose(self):
        self.obj_pose.position.x = np.mean(self.remove_outlier(self.ox))
        self.obj_pose.position.y = np.mean(self.remove_outlier(self.oy))
        self.navigation_pose.position.x = np.mean(self.remove_outlier(self.nx))
        self.navigation_pose.position.y = np.mean(self.remove_outlier(self.ny))

    def calculate_rotations(self):
        angle = np.mean(self.remove_outlier(self.na))
        self.navigation_pose.orientation = self.euler_to_quaternion(angle)

    def quaternion_to_euler(self, pose):
        angle = tf.transformations.euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )[2]
        if angle < 0:
            angle += 2 * math.pi
        return angle

    def euler_to_quaternion(self, angle):
        return Quaternion(*list(tf.transformations.quaternion_from_euler(0, 0, angle)))

    def remove_outlier(self, array, max_deviation=0.5):
        mean = np.mean(array)
        std = np.std(array)
        distance_from_mean = abs(array - mean)
        not_outlier = distance_from_mean < max_deviation * std
        filtered = np.array(array)[not_outlier]
        if len(filtered) == 0:
            filtered = [array[0]]
        return filtered


if __name__ == "__main__":
    RingSegmentation()
