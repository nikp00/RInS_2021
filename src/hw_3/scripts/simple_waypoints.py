#!/usr/bin/env python


import rospy
import math
import random
import sys
import json
import tf

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from move_base_msgs.msg import MoveBaseActionResult
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class SimpleWaypoints:
    def __init__(self, waypoints_file):
        self.node = rospy.init_node("simple_waypoints")
        self.pose_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_sub_callback
        )

        self.marker_publisher = rospy.Publisher(
            "/waypoint_marker", Marker, queue_size=10
        )

        self.waypoints = json.load(open(waypoints_file))

        self.waypoint_status = 3
        self.seq = 0

        self.run()

    def run(self):
        r = rospy.Rate(1)
        while self.pose_publisher.get_num_connections() <= 0:
            print(self.pose_publisher.get_num_connections())
            r.sleep()

        while not rospy.is_shutdown() and self.seq < len(self.waypoints):
            if self.waypoint_status == 3:
                self.waypoint_status = 0
                self.send_next_waypoint()
            r.sleep()

    def send_next_waypoint(self):
        waypoint = self.waypoints[self.seq]
        self.send_marker(self.seq, waypoint)

        msg = PoseStamped()
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.position = Point(*waypoint["position"])

        msg.pose.orientation = Quaternion(
            *list(
                tf.transformations.quaternion_from_euler(
                    0, 0, waypoint["orientation"][2]
                )
            )
        )

        self.pose_publisher.publish(msg)
        print(msg)
        print(self.pose_publisher.get_num_connections())

    def send_marker(
        self,
        id,
        waypoint,
        sx=0.5,
        sy=0.5,
        r=0,
        g=0,
        b=0,
        a=1,
        mType=Marker.SPHERE,
        action=Marker.ADD,
        lifetime=0,
    ):
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        msg.ns = "waypoints"
        msg.id = id
        msg.type = mType
        msg.action = action

        point = Point(*waypoint["position"])

        msg.pose.position = point
        msg.scale.x = sx
        msg.scale.y = sy
        msg.scale.z = 0

        msg.color.r = r
        msg.color.g = g
        msg.color.b = b
        msg.color.a = a

        msg.lifetime = rospy.Duration(lifetime)

        self.marker_publisher.publish(msg)

    def result_sub_callback(self, data):
        res_status = data.status.status
        print(f"status: {res_status}")
        if res_status == 3:
            self.waypoint_status = 3
            self.send_marker(
                self.seq,
                self.waypoints[self.seq],
                a=0.2,
                action=Marker.MODIFY,
            )

            self.seq += 1
            print("Goal reached", res_status)


if __name__ == "__main__":
    sw = SimpleWaypoints(sys.argv[1])
