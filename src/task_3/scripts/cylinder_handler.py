import rospy
import copy
import tf2_geometry_msgs
import tf2_ros
import tf
import numpy as np
from collections import defaultdict


from visualization_msgs.msg import MarkerArray, Marker

from task_3.srv import ColorClassifierService, ColorClassifierServiceRequest
from geometry_msgs.msg import (
    Point,
    Vector3,
    PoseStamped,
    Pose,
    Quaternion,
)
from std_msgs.msg import ColorRGBA
from task_3.msg import CylinderSegmentation, PoseAndColor, PoseAndColorArray


class CylinderHandler:
    def __init__(self):
        self.node = rospy.init_node("cylinder_handler")

        self.distance_threshold = 0.2

        self.cylinders = list()
        self.seq = 1
        self.sent_ids = list()

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # Publishers
        self.cylinder_marker_publisher = rospy.Publisher(
            "cylinder_markers", MarkerArray, queue_size=10
        )
        self.n_detections_marker_publisher = rospy.Publisher(
            "cylinder_n_detections_markers", MarkerArray, queue_size=10
        )
        self.cylinder_pose_publisher = rospy.Publisher(
            "cylinder_pose", PoseAndColorArray, queue_size=10
        )

        # Services
        self.get_color = rospy.ServiceProxy("/color_classifier", ColorClassifierService)
        rospy.wait_for_service("/color_classifier")

        # Subscribers
        self.pose_subscriber = rospy.Subscriber(
            "detected_cylinder", CylinderSegmentation, self.cylinder_callback
        )

        rospy.spin()

    def get_current_pose(self, time) -> Pose:
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
        return pose.pose

    def calculate_direction_vector(self, x1, y1, x2, y2):
        direction = (x1 - x2, y1 - y2)
        magnitude = np.sqrt(np.power(direction[0], 2) + np.power(direction[1], 2))

        dx = direction[0] / magnitude
        dy = direction[1] / magnitude

        return dx, dy

    def cylinder_callback(self, data: CylinderSegmentation):
        color = data.color
        point = data.point

        req = ColorClassifierServiceRequest()
        req.mode = 1
        req.color = color
        res = self.get_color(req)

        print(res.color, color.r, color.g, color.b)

        base_pose = self.get_current_pose(point.header.stamp)
        angle = tf.transformations.euler_from_quaternion(
            [
                base_pose.orientation.x,
                base_pose.orientation.y,
                base_pose.orientation.z,
                base_pose.orientation.w,
            ]
        )[2]

        angle = np.arctan2(
            point.point.y - base_pose.position.y,
            point.point.x - base_pose.position.x,
        )

        pose = Pose()
        pose.position = point.point

        dx, dy = self.calculate_direction_vector(
            base_pose.position.x, base_pose.position.y, point.point.x, point.point.y
        )
        navigation_pose = Pose()
        navigation_pose.position.x = point.point.x + dx * 0.4
        navigation_pose.position.y = point.point.y + dy * 0.4
        navigation_pose.orientation = Quaternion(
            *list(tf.transformations.quaternion_from_euler(0, 0, angle))
        )

        if np.isnan(pose.position.x) or res.color == "black":
            return

        pose.position.z = 0

        skip = False
        for e in self.cylinders:
            if (
                np.sqrt(
                    np.power(pose.position.x - e.obj_pose.position.x, 2)
                    + np.power(pose.position.y - e.obj_pose.position.y, 2)
                )
                < 0.5
            ):
                e.add_pose(pose, navigation_pose)
                e.color_name[res.color] += 1
                e.color[(res.marker_color.r, res.marker_color.g, res.marker_color.b)] += 1
                e.n_detections += 1
                skip = True
                break

        if not skip:
            cylinder = Cylinder(
                pose,
                navigation_pose,
                (res.marker_color.r, res.marker_color.g, res.marker_color.b),
                res.color,
                self.seq,
            )
            self.seq += 1
            self.cylinders.append(cylinder)

        self.cylinder_marker_publisher.publish(
            [e.to_marker() for e in self.cylinders]
            + [e.to_navigation_marker() for e in self.cylinders]
        )

        self.n_detections_marker_publisher.publish([e.to_text() for e in self.cylinders])
        self.cylinder_pose_publisher.publish(
            PoseAndColorArray(
                [
                    PoseAndColor(e.obj_pose, e.navigation_pose, e.get_color_name(), e.id)
                    for e in self.cylinders
                ]
            )
        )


class Cylinder:
    def __init__(
        self, obj_pose: Pose, navigation_pose: Pose, color: tuple, color_name: str, id: int
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
        m.ns = "cylinder_marker"
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose = copy.deepcopy(self.obj_pose)
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
        if self.n_detections > 1:
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
        m.ns = "cylinder_n_detections_markers"
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose = copy.deepcopy(self.obj_pose)
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
        return tf.transformations.euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )[2]

    def euler_to_quaternion(self, angle):
        return Quaternion(*list(tf.transformations.quaternion_from_euler(0, 0, angle)))

    def remove_outlier(self, array, max_deviation=2):
        mean = np.mean(array)
        std = np.std(array)
        distance_from_mean = abs(array - mean)
        not_outlier = distance_from_mean < max_deviation * std
        filtered = np.array(array)[not_outlier]
        if len(filtered) == 0:
            filtered = [array[0]]
        return filtered


if __name__ == "__main__":
    CylinderHandler()
