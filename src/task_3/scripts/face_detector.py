import sys

from numpy.core.fromnumeric import shape
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import tf
import face_recognition
import copy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import (
    PointStamped,
    Vector3,
    Pose,
    Quaternion,
    PoseStamped,
    Point,
)
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from task_3.msg import FaceData, FaceDataArray
from task_3.srv import GetNormalService, GetNormalServiceRequest, GetNormalServiceResponse


class FaceDetectorDNN:
    def __init__(self, dnn_args):
        self.node = rospy.init_node("face_detector")
        self.bridge = CvBridge()

        self.face_net = cv2.dnn.readNetFromCaffe(*dnn_args)

        self.dims = (0, 0, 0)
        self.seq = 0

        self.face_marker_publisher = rospy.Publisher("face_markers", MarkerArray, queue_size=1000)
        self.n_detections_marker_publisher = rospy.Publisher(
            "face_n_detections_markers", MarkerArray, queue_size=10
        )
        self.face_pose_publisher = rospy.Publisher("face_pose", FaceDataArray, queue_size=10)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # Services
        self.get_normal = rospy.ServiceProxy("/get_normal", GetNormalService)
        rospy.wait_for_service("/get_normal")

        self.faces = list()
        self.rgb_image = None

        rospy.sleep(3)
        print("Start")

        self.run()

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            faces = self.find_faces()
            self.process_faces(faces)
            r.sleep()

    def camera_to_world_point(self, x, y, nx=0, nz=0, stamp=False):
        if not stamp:
            stamp = rospy.Time.now()
        point_s = PointStamped()
        point_s.point.x = -y + (nx * 0.05 * 10)
        point_s.point.y = 0
        point_s.point.z = x + (nz * 0.05 * 10)
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = rospy.Time.now()

        pose = None
        while pose == None:
            try:
                point_world = self.tf_buf.transform(point_s, "map")
                pose = Pose()
                pose.position.x = point_world.point.x
                pose.position.y = point_world.point.y
                pose.position.z = point_world.point.z
            except Exception as e:
                print(e)
        return pose

    def get_pose(self, coords, dist, stamp):
        k_f = 554
        x1, x2, y1, y2 = coords
        face_x = self.dims[1] / 2 - (x1 + x2) / 2.0

        res = self.get_normal(x=(x1 + x2) // 2, y=(y1 + y2) // 2, radius=0.5)

        angle_to_target = np.arctan2(face_x, k_f)

        x, y = dist * np.cos(angle_to_target), dist * np.sin(angle_to_target)

        face_pose = self.camera_to_world_point(x, y, stamp=stamp)
        navigation_pose = self.camera_to_world_point(x, y, nx=res.nx, nz=res.nz, stamp=stamp)

        angle = np.arctan2(
            navigation_pose.position.y - face_pose.position.y,
            navigation_pose.position.x - face_pose.position.x,
        )

        navigation_pose.orientation = Quaternion(
            *list(tf.transformations.quaternion_from_euler(0, 0, angle))
        )

        return face_pose, navigation_pose

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

    def find_faces(self):
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        blob = cv2.dnn.blobFromImage(
            cv2.resize(self.rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0)
        )
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()

        return face_detections

    def process_faces(self, faces):
        try:
            depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        self.dims = self.rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]
        for i in range(0, faces.shape[2]):
            confidence = faces[0, 0, i, 2]
            if confidence > 0.5:
                box = faces[0, 0, i, 3:7] * np.array([w, h, w, h])
                box = box.astype("int")
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
                if any(e < 0 for e in (x1, y1, x2, y2)):
                    continue

                face_region = self.rgb_image[y1:y2, x1:x2]

                face_distance = float(np.nanmean(depth_image[y1:y2, x1:x2]))

                depth_time = depth_image_message.header.stamp

                face_pose, navigation_pose = self.get_pose(
                    (x1, x2, y1, y2), face_distance, depth_time
                )

                if face_pose is not None:
                    enc = face_recognition.face_encodings(self.rgb_image[y1:y2, x1:x2])
                    enc_available = len(enc) > 0
                    if enc_available:
                        enc = enc[0]
                    else:
                        enc = np.zeros(shape=(128,))
                    skip = False
                    for e in self.faces:
                        if np.sqrt(
                            np.power(face_pose.position.x - e.face_pose.position.x, 2)
                            + np.power(face_pose.position.y - e.face_pose.position.y, 2)
                        ) < 0.5 and (
                            not enc_available or face_recognition.compare_faces([e.enc], enc)[0]
                        ):
                            e.color = ColorRGBA(0, 1, 1, 1)
                            e.add_pose(face_pose, navigation_pose)
                            e.calculate_pose()
                            e.n_detections += 1
                            skip = True
                            break

                    if not skip:
                        self.seq += 1
                        self.faces.append(Face(face_pose, navigation_pose, enc, self.seq))

        self.face_marker_publisher.publish([e.to_marker() for e in self.faces])
        self.n_detections_marker_publisher.publish([e.to_text() for e in self.faces])

        self.face_pose_publisher.publish(
            FaceDataArray(
                [FaceData(e.navigation_pose, e.face_pose, e.mask, e.id) for e in self.faces]
            )
        )


class Face:
    def __init__(self, face_pose: Pose, navigation_pose: Pose, enc, id):
        self.face_pose = face_pose
        self.navigation_pose = navigation_pose
        self.enc = copy.deepcopy(enc)
        self.color = ColorRGBA(0, 1, 0, 1)
        self.id = id
        self.fx = [face_pose.position.x]
        self.fy = [face_pose.position.y]
        self.nx = [navigation_pose.position.x]
        self.ny = [navigation_pose.position.y]
        self.n_detections = 1
        self.mask = False

    def add_pose(self, face_pose: Pose, navigation_pose: Pose):
        self.fx.append(face_pose.position.x)
        self.fx.append(face_pose.position.y)
        self.nx.append(navigation_pose.position.x)
        self.nx.append(navigation_pose.position.x)

    def to_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration(0)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.pose = self.face_pose
        marker.color = self.color
        marker.id = self.id
        return marker

    def to_text(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()

        m.id = self.id
        m.ns = "face_n_detections_markers"
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose = copy.deepcopy(self.face_pose)
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

    def calculate_pose(self):
        self.face_pose.position.x = np.mean(self.fx)
        self.face_pose.position.y = np.mean(self.fy)
        self.navigation_pose.position.x = np.mean(self.nx)
        self.navigation_pose.position.y = np.mean(self.ny)


if __name__ == "__main__":
    fd = FaceDetectorDNN(sys.argv[1:3])
