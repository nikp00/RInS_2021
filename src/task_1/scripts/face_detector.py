import sys
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


class FaceDetectorDNN:
    def __init__(self, dnn_args):
        self.node = rospy.init_node("face_detector")
        self.bridge = CvBridge()

        self.face_net = cv2.dnn.readNetFromCaffe(*dnn_args)

        self.dims = (0, 0, 0)
        self.marker_array = MarkerArray()
        self.marker_num = 0

        self.markers_pub = rospy.Publisher("face_markers", MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.faces = list()

        rospy.sleep(3)
        print("Start")

    def get_pose(self, coords, dist, stamp):
        k_f = 554

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1 + x2) / 2.0
        face_y = self.dims[0] / 2 - (y1 + y2) / 2.0

        angle_to_target = np.arctan2(face_x, k_f)

        x, y = dist * np.cos(angle_to_target), dist * np.sin(angle_to_target)

        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        try:
            point_world = self.tf_buf.transform(point_s, "map")
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
        except Exception as e:
            print(e)
            pose = None

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

    def find_faces(self):
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message(
                "/camera/depth/image_raw", Image
            )
        except Exception as e:
            print(e)
            return 0

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]

        blob = cv2.dnn.blobFromImage(
            cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0)
        )
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()

        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence > 0.5:
                box = face_detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                box = box.astype("int")
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
                if any(e < 0 for e in (x1, y1, x2, y2)):
                    continue

                # Extract region containing face
                face_region = rgb_image[y1:y2, x1:x2]

                # Find the distance to the detected face
                face_distance = float(np.nanmean(depth_image[y1:y2, x1:x2]))

                # Get the time that the depth image was recieved
                depth_time = depth_image_message.header.stamp

                # Find the location of the detected face
                pose = self.get_pose((x1, x2, y1, y2), face_distance, depth_time)
                if pose is not None:
                    enc = face_recognition.face_encodings(rgb_image[y1:y2, x1:x2])
                    if len(enc) == 0:
                        continue
                    enc = enc[0]
                    skip = False
                    for e in self.faces:
                        if (
                            np.sqrt(
                                np.power(pose.position.x - e.pose.position.x, 2)
                                + np.power(pose.position.y - e.pose.position.y, 2)
                            )
                            < 0.5
                            and face_recognition.compare_faces([e.enc], enc)[0]
                        ):
                            e.pose.position.x = float(
                                (pose.position.x + e.pose.position.x) / 2
                            )
                            e.pose.position.y = float(
                                (pose.position.y + e.pose.position.y) / 2
                            )
                            e.color = ColorRGBA(0, 1, 1, 1)

                            skip = True
                            break
                    if not skip:
                        self.marker_num += 1
                        self.faces.append(Face(pose, enc, self.marker_num))
                        print(len(self.faces))

        self.marker_array.markers = list()

        for e in self.faces:
            self.marker_array.markers.append(e.to_marker())

        self.markers_pub.publish(self.marker_array)


class Face:
    def __init__(self, pose, enc, mId):
        self.pose = pose
        self.enc = copy.deepcopy(enc)
        self.color = ColorRGBA(0, 1, 0, 1)
        self.id = mId

    def to_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration(0)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.pose = self.pose
        marker.color = self.color
        marker.id = self.id
        return marker


if __name__ == "__main__":
    fd = FaceDetectorDNN(sys.argv[1:3])

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        fd.find_faces()
        r.sleep()
