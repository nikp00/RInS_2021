import sys
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import tf

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, Quaternion
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
            pose.orientation = Quaternion(
                *list(tf.transformations.quaternion_from_euler(0, 0, angle_to_target))
            )
        except Exception as e:
            print(e)
            pose = None
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

                # Extract region containing face
                face_region = rgb_image[y1:y2, x1:x2]

                # Visualize the extracted face
                # cv2.imshow("ImWindow", face_region)
                # cv2.waitKey(1)

                # Find the distance to the detected face
                face_distance = float(np.nanmean(depth_image[y1:y2, x1:x2]))

                # Get the time that the depth image was recieved
                depth_time = depth_image_message.header.stamp

                # Find the location of the detected face
                pose = self.get_pose((x1, x2, y1, y2), face_distance, depth_time)
                if pose is not None:
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.frame_locked = False
                    marker.lifetime = rospy.Duration(0)
                    marker.scale = Vector3(0.1, 0.1, 0.1)

                    skip = False
                    for m in self.marker_array.markers:
                        if (
                            np.sqrt(
                                np.power(pose.position.x - m.pose.position.x, 2)
                                + np.power(pose.position.y - m.pose.position.y, 2)
                            )
                            < 0.5
                        ):
                            m.action = Marker.MODIFY
                            m.pose.position.x = float(
                                (pose.position.x + m.pose.position.x) / 2
                            )
                            m.pose.position.y = float(
                                (pose.position.y + m.pose.position.y) / 2
                            )
                            m.color = ColorRGBA(0, 1, 1, 1)

                            print("Same face")
                            skip = True
                            break
                    if not skip:
                        print("New face")
                        self.marker_num += 1
                        marker.color = ColorRGBA(0, 1, 0, 1)
                        marker.header.stamp = rospy.Time(0)
                        marker.pose = pose
                        marker.id = self.marker_num
                        self.marker_array.markers.append(marker)
                    self.markers_pub.publish(self.marker_array)


if __name__ == "__main__":
    fd = FaceDetectorDNN(sys.argv[1:3])

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        fd.find_faces()
        r.sleep()
