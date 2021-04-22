import rospy
import numpy as np
import random

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import (
    PoseArray,
    Pose,
    Point,
    Quaternion,
    TransformStamped,
    PointStamped,
)
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import Image


class MapSegmentation:
    def __init__(self):
        self.node = rospy.init_node("map_segmentation")
        self.img_publisher = rospy.Publisher(
            "/map_segmentation_img", Image, queue_size=10
        )
        self.waypoint_publisher = rospy.Publisher(
            "/waypoints", PoseArray, queue_size=10
        )

        self.quads = None
        self.grouped_quads = list()
        self.map = None
        self.waypoints = PoseArray()
        self.img = Image()
        self.n_connections = {"img": 0, "waypoints": 0}

        self.segmentation_size = rospy.get_param("~segmentation_size")
        self.min_quad_size = rospy.get_param("~min_quad_size")

        self.map_msg = rospy.wait_for_message("/map", OccupancyGrid)

        self.occupancy_grid_to_img()
        self.create_grid()
        self.group_quadrants()
        self.generate_waypoints()
        self.img_to_msg()

    def publish(self):
        n = self.waypoint_publisher.get_num_connections()
        if n > self.n_connections["waypoints"]:
            self.n_connections["waypoints"] = n
        else:
            self.n_connections["waypoints"] = n

        self.waypoint_publisher.publish(self.waypoints)
        n = self.img_publisher.get_num_connections()
        if n > self.n_connections["img"]:
            self.n_connections["img"] = n
            self.img_publisher.publish(self.img)
        else:
            self.n_connections["img"] = n

    def img_to_msg(self):
        self.img.height = self.map_msg.info.height
        self.img.width = self.map_msg.info.width
        self.img.encoding = "rgb8"
        self.img.step = 3 * self.map_msg.info.width
        self.img.data = self.map.reshape(
            (self.map_msg.info.width * self.map_msg.info.height * 3)
        ).tolist()
        print("OK")

    def occupancy_grid_to_img(self):
        self.map = np.flip(
            np.array([[e, e, e] for e in self.map_msg.data], dtype=np.uint8).reshape(
                (self.map_msg.info.height, self.map_msg.info.width, 3)
            ),
            axis=0,
        )

    def create_grid(self):
        self.quads = [
            ((x, y), (x + self.segmentation_size, y + self.segmentation_size))
            for x in range(0, self.map.shape[0], self.segmentation_size)
            for y in range(0, self.map.shape[0], self.segmentation_size)
        ]

    def group_quadrants(self):
        closed = set()
        for i, ((y0, x0), (y1, x1)) in enumerate(self.quads):
            if self.check_pure(y0, y1, x0, x1):
                if (y0, y1, x1, x1 + self.segmentation_size) in closed:
                    continue

                span = 1
                while self.check_pure(
                    y0,
                    y1 + span * self.segmentation_size,
                    x0,
                    x1 + span * self.segmentation_size,
                ):
                    span += 1
                span -= 1

                if span >= self.min_quad_size:
                    self.map[
                        y0 : y1 + span * self.segmentation_size,
                        x0 : x1 + span * self.segmentation_size,
                    ] = [
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 100),
                    ]
                    self.grouped_quads.append(
                        (
                            (y0, x0),
                            (
                                y1 + span * self.segmentation_size,
                                x1 + span * self.segmentation_size,
                            ),
                        )
                    )
                else:
                    self.map[y0:y1, x0:x1] = [0, 0, 255]

    def generate_waypoints(self):
        map_transform = TransformStamped()
        map_transform.transform.translation.x = self.map_msg.info.origin.position.x
        map_transform.transform.translation.y = self.map_msg.info.origin.position.y
        map_transform.transform.translation.z = self.map_msg.info.origin.position.z
        map_transform.transform.rotation = self.map_msg.info.origin.orientation

        for (y0, x0), (y1, x1) in self.grouped_quads:
            my = (
                self.map_msg.info.height - (y0 + (y1 - y0) / 2)
            ) * self.map_msg.info.resolution

            mx = (x0 + (x1 - x0) / 2) * self.map_msg.info.resolution
            pt = PointStamped()
            pt.point.x = mx
            pt.point.y = my
            pt.point.z = 0
            pt = do_transform_point(pt, map_transform)

            pose = Pose()
            pose.position = pt.point

            self.waypoints.poses.append(pose)

    def check_pure(self, y0, y1, x0, x1):
        return all(all(e == [0, 0, 0]) for row in self.map[y0:y1, x0:x1] for e in row)


if __name__ == "__main__":
    ms = MapSegmentation()

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        ms.publish()
        r.sleep()