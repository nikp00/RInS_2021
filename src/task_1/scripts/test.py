#!/usr/bin/env python


import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import tf
import tf2_ros

import random

import json

import copy


from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, Point, PointStamped
from tf2_geometry_msgs import do_transform_point


class MapDisplayer:
    def __init__(self):
        self.node = rospy.init_node("simple_waypoints")
        costmap = rospy.wait_for_message(
            "/move_base/global_costmap/costmap", OccupancyGrid
        )
        map_ = rospy.wait_for_message("/map", OccupancyGrid)

        map_img = self.occupancy_grid_to_img(map_)

        # map_img_10, quad_10 = self.create_grid(map_img, 10)
        # map_img_20, quad_20 = self.create_grid(map_img, 20)
        # map_img_40, quad_40 = self.create_grid(map_img, 40)

        # cv2.imshow("map10", map_img_10)
        # cv2.imshow("map20", map_img_20)
        # cv2.imshow("map40", map_img_40)

        quad_size = 1

        _, quads = self.create_grid(map_img, quad_size)
        map_test, grouped_quads = self.group_quadrants(map_img, quads, quad_size, 15)
        map_test1, quads = self.create_grid(map_test, quad_size)

        print(map_.info)

        self.export_waypoints(grouped_quads, map_)

        cv2.imshow("segmented map", map_test)
        cv2.imshow("segmented map1", map_test1)
        cv2.imshow("map", map_img)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            print(1)
            r.sleep()

    def occupancy_grid_to_img(self, og):

        return np.flip(
            np.array([[e, e, e] for e in og.data], dtype=np.uint8).reshape(
                (480, 480, 3)
            ),
            axis=0,
        )

    def create_grid(self, map_img, size):
        map_img = copy.deepcopy(map_img)
        for i in range(0, map_img.shape[0], size):
            for j in range(0, map_img.shape[0], size):
                map_img[i, :] = [255, 0, 0]
                map_img[:, j] = [255, 0, 0]
        rects = [
            ((x, y), (x + size, y + size))
            for x in range(0, map_img.shape[0], size)
            for y in range(0, map_img.shape[0], size)
        ]
        return map_img, rects

    def group_quadrants(self, map_img, quads, size, min_quad_size=2):
        map_img = copy.deepcopy(map_img)
        closed = set()
        grouped_quads = list()
        for i, ((y0, x0), (y1, x1)) in enumerate(quads):
            if self.check_pure(map_img, y0, y1, x0, x1):
                if (y0, y1, x1, x1 + size) in closed:
                    continue

                span = 1
                while self.check_pure(
                    map_img, y0, y1 + span * size, x0, x1 + span * size
                ):
                    span += 1
                span -= 1
                if span >= 1 and span >= min_quad_size:
                    map_img[y0 : y1 + span * size, x0 : x1 + span * size] = [
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255),
                    ]
                    grouped_quads.append(
                        ((y0, x0), (y1 + span * size, x1 + span * size))
                    )
                else:
                    # grouped_quads.append(((y0, x0), (y1, x1)))
                    map_img[y0:y1, x0:x1] = [0, 0, 255]
        return map_img, grouped_quads

        # if self.check_pure(map_img, y0, y1 + size, x0, x1 + size):
        #     map_img[y0 : y1 + size, x0 : x1 + size] = [
        #         random.randint(0, 255),
        #         random.randint(0, 255),
        #         random.randint(0, 255),
        #     ]
        #     closed.add((y0, y1, x1, x1 + size))
        #     closed.add((y0, y1, x0, x1))
        #     closed.add((y1, y1 + size, x1, x1 + size))
        # elif self.check_pure(map_img, y0, y1, x1, x1 + size):
        #     map_img[y0:y1, x0 : x1 + size] = [
        #         random.randint(0, 255),
        #         random.randint(0, 255),
        #         random.randint(0, 255),
        #     ]
        #     closed.add((y0, y1, x1, x1 + size))
        # else:
        #     map_img[y0:y1, x0:x1] = [0, 0, 255]

    def check_pure(self, map_img, y0, y1, x0, x1):
        return all(all(e == [0, 0, 0]) for row in map_img[y0:y1, x0:x1] for e in row)

    def export_waypoints(self, quads, map_):

        map_transform = TransformStamped()
        map_transform.transform.translation.x = map_.info.origin.position.x
        map_transform.transform.translation.y = map_.info.origin.position.y
        map_transform.transform.translation.z = map_.info.origin.position.z
        map_transform.transform.rotation = map_.info.origin.orientation

        waypoints = list()
        for (y0, x0), (y1, x1) in quads:
            my = (480 - (y0 + (y1 - y0) / 2)) * map_.info.resolution
            mx = (x0 + (x1 - x0) / 2) * map_.info.resolution
            pt = PointStamped()
            pt.point.x = mx
            pt.point.y = my
            pt.point.z = 0

            pt = do_transform_point(pt, map_transform)
            entry = {"position": [pt.point.x, pt.point.y, 0], "orientation": [0, 0, 0]}
            waypoints.append(entry)
        json.dump(waypoints, open("waypoints.json", "w"))


if __name__ == "__main__":
    MapDisplayer()
