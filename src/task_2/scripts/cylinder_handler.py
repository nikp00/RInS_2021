import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Quaternion
import tf

import numpy as np


class CylinderHandler:
    def __init__(self):
        self.node = rospy.init_node("cylinder_handler")

        self.distance_threshold = 0.2

        self.markers = MarkerArray()
        self.seq = 0

        self.cylinder_marker_publisher = rospy.Publisher(
            "cylinder_markers", MarkerArray, queue_size=10
        )

        self.pose_subscriber = rospy.Subscriber(
            "detected_cylinder", PointStamped, self.cylinder_callback
        )

        rospy.spin()

    def cylinder_callback(self, data: PointStamped):
        print("-------------------------")
        print(data.point)
        cylinder = PoseStamped()
        cylinder.pose.position = data.point
        cylinder.pose.orientation = Quaternion(0, 0, 0, 1)

        skip = False
        for m in self.markers.markers:
            if (
                np.sqrt(
                    np.power(cylinder.pose.position.x - m.pose.position.x, 2)
                    + np.power(cylinder.pose.position.y - m.pose.position.y, 2)
                )
                < 0.5
            ):
                m.pose.position.x = float(
                    (m.pose.position.x + cylinder.pose.position.x) / 2
                )
                m.pose.position.y = float(
                    (m.pose.position.y + cylinder.pose.position.y) / 2
                )
                m.pose.position.z = float(
                    (m.pose.position.z + cylinder.pose.position.z) / 2
                )

                m.color.r += 10
                skip = True
                print(
                    np.sqrt(
                        np.power(cylinder.pose.position.x - m.pose.position.x, 2)
                        + np.power(cylinder.pose.position.y - m.pose.position.y, 2)
                    )
                )
                break

        if not skip:
            print("Got new cylinder")
            self.markers.markers.append(
                self.create_marker(
                    self.seq,
                    cylinder.pose,
                    ns="cylinder_marker",
                    frame_id=data.header.frame_id,
                    mType=Marker.CYLINDER,
                    sx=0.1,
                    sy=0.1,
                    sz=0.1,
                    g=255,
                )
            )
            self.seq += 1

        self.cylinder_marker_publisher.publish(self.markers)

    def create_marker(
        self,
        id,
        pose: Pose,
        sx=0.5,
        sy=0.5,
        sz=0,
        r=0,
        g=0,
        b=0,
        a=1,
        mType=Marker.SPHERE,
        action=Marker.ADD,
        text="",
        lifetime=0,
        ns="",
        frame_id="map",
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()

        m.ns = ns
        m.id = id
        m.type = mType
        m.action = action

        m.pose.position = pose.position
        m.pose.orientation = pose.orientation
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz

        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = a

        m.lifetime = rospy.Duration(lifetime)

        m.text = text

        return m


if __name__ == "__main__":
    CylinderHandler()
