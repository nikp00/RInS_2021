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
    PointStamped,
    Point,
    PoseStamped,
    Pose,
    Quaternion,
    PoseArray,
)
from std_msgs.msg import ColorRGBA, Header
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
            base_pose.position.y - point.point.y,
            base_pose.position.x - point.point.x,
        )

        pose = PoseStamped()
        pose.pose.position = point.point
        pose.pose.orientation = Quaternion(
            *list(tf.transformations.quaternion_from_euler(0, 0, angle))
        )

        if np.isnan(pose.pose.position.x) or res.color == "black":
            return

        pose.pose.position.z = 0

        skip = False
        for e in self.cylinders:
            if (
                np.sqrt(
                    np.power(pose.pose.position.x - e.pose.pose.position.x, 2)
                    + np.power(pose.pose.position.y - e.pose.pose.position.y, 2)
                )
                < 0.5
            ):
                # e.pose.pose.position.x = float(
                #     (e.pose.pose.position.x + pose.pose.position.x) / 2
                # )
                # e.pose.pose.position.y = float(
                #     (e.pose.pose.position.y + pose.pose.position.y) / 2
                # )
                # e.pose.pose.position.z = float(
                #     (e.pose.pose.position.z + pose.pose.position.z) / 2
                # )
                e.x.append(pose.pose.position.x)
                e.y.append(pose.pose.position.y)
                e.angle.append(angle)
                e.calculate_pose()

                e.pose.pose.orientation = pose.pose.orientation

                e.color_name[res.color] += 1
                e.color[(res.marker_color.r, res.marker_color.g, res.marker_color.b)] += 1

                e.n_detections += 1
                skip = True
                break

        if not skip:
            cylinder = Cylinder(
                pose,
                angle,
                (res.marker_color.r, res.marker_color.g, res.marker_color.b),
                res.color,
                self.seq,
            )
            self.seq += 1
            self.cylinders.append(cylinder)

        self.cylinder_marker_publisher.publish([e.to_marker() for e in self.cylinders])
        self.n_detections_marker_publisher.publish([e.to_text() for e in self.cylinders])

        poses = list()

        for e in self.sent_ids:
            for x in self.cylinders:
                if x.id == e:
                    poses.append(PoseAndColor(x.pose.pose, x.get_color_name(), x.id))

        for e in self.cylinders:
            if e.id not in self.sent_ids and e.n_detections > 1:
                poses.append(PoseAndColor(e.pose.pose, e.get_color_name(), e.id))
                self.sent_ids.append(e.id)

        self.cylinder_pose_publisher.publish(PoseAndColorArray(poses))


class Cylinder:
    def __init__(self, pose: PoseStamped, angle, color: tuple, color_name: str, id: int):
        self.pose = pose
        # self.color = Cylinder.colors["yellow"]  # color
        self.x = [pose.pose.position.x]
        self.y = [pose.pose.position.y]
        self.angle = [angle]
        self.color = defaultdict(int)
        self.color[color] += 1
        self.id = id
        self.n_detections = 1
        self.color_name = defaultdict(int)
        self.color_name[color_name] += 1

    def to_marker(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()

        m.id = self.id
        m.ns = "cylinder_marker"
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose = copy.deepcopy(self.pose.pose)
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
        m.ns = "cylinder_n_detections_markers"
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose = copy.deepcopy(self.pose.pose)
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
        self.pose.pose.position.x = np.mean(self.x)
        self.pose.pose.position.y = np.mean(self.y)
        # self.pose.pose.orientation = Quaternion(
        #     *list(tf.transformations.quaternion_from_euler(0, 0, np.mean(self.angle)))
        # )
        # print(self.angle)


if __name__ == "__main__":
    CylinderHandler()
