#!/usr/bin/env python


import rospy
import math
import random
import sys
import json
import tf

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult


class SimpleWaypoints:
    def __init__(self, waypoints_file):
        self.node = rospy.init_node("simple_waypoints")
        self.pose_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_sub_callback
        )
        self.status_sub = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.status_sub_callback
        )
        self.waypoints = iter(
            [(i, e) for i, e in enumerate(json.load(open(waypoints_file)))]
        )

        self.waypoint_status = 3
        self.seq = 0
        self.res_status = 0

        self.run()

    def run(self):
        r = rospy.Rate(1)
        while self.pose_publisher.get_num_connections() <= 0:
            print(self.pose_publisher.get_num_connections())
            r.sleep()

        while not rospy.is_shutdown() and self.res_status != -1:
            if self.waypoint_status == 3:
                self.waypoint_status = 0
                self.send_next_waypoint()
            r.sleep()

    def send_next_waypoint(self):
        self.seq, waypoint = next(self.waypoints, (-1, None))
        if waypoint != None:
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

    def status_sub_callback(self, data):
        pass

    def result_sub_callback(self, data):
        self.res_status = data.status.status
        print(f"status: {self.res_status}")
        if self.res_status == 3:
            self.waypoint_status = 3
            if self.seq == -1:
                self.res_status = -1
            print("Goal reached", self.res_status)


if __name__ == "__main__":
    sw = SimpleWaypoints(sys.argv[1])
