#!/usr/bin/env python3


import rospy
import math
import random
import sys
import json
import tf

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from move_base_msgs.msg import MoveBaseActionResult
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan


class SimpleWaypoints:
    def __init__(self, waypoints_file):
        self.node = rospy.init_node("simple_waypoints")
        self.pose_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.marker_publisher = rospy.Publisher(
            "/waypoint_marker", Marker, queue_size=10
        )
        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_sub_callback
        )
        self.current_pose_sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.current_pose_callback
        )

        self.get_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        self.waypoints = json.load(open(waypoints_file))
        self.current_pose = None

        self.waypoint_status = 3
        self.seq = 0

        self.waypoints_history = list()

        self.run()

    def run(self):
        r = rospy.Rate(1)
        while self.pose_publisher.get_num_connections() <= 0:
            print(self.pose_publisher.get_num_connections())
            r.sleep()

        while self.current_pose == None:
            r.sleep()

        while not rospy.is_shutdown() and len(self.waypoints) > 0:
            if self.waypoint_status == 3:
                self.waypoint_status = 0
                self.send_next_waypoint()
            r.sleep()

    def find_nearest_waypoint(self):
        start = PoseStamped()
        start.header.seq = 0
        start.header.stamp = rospy.Time.now()
        start.header.frame_id = "map"
        start.pose.position = self.current_pose.position
        start.pose.orientation = self.current_pose.orientation

        min_len = 100000

        for e in self.waypoints:
            goal = PoseStamped()
            goal.header.seq = 0
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position = Point(*e["position"])
            goal.pose.orientation = Quaternion(
                *list(
                    tf.transformations.quaternion_from_euler(0, 0, e["orientation"][2])
                )
            )

            req = GetPlan()
            req.start = start
            req.goal = goal
            req.tolerance = 0.5
            resp = self.get_plan(req.start, req.goal, req.tolerance)
            path_len = sum(
                [
                    math.sqrt(
                        pow(
                            (
                                resp.plan.poses[i + 1].pose.position.x
                                - resp.plan.poses[i].pose.position.x
                            ),
                            2,
                        )
                        + pow(
                            (
                                resp.plan.poses[i + 1].pose.position.y
                                - resp.plan.poses[i].pose.position.y
                            ),
                            2,
                        )
                    )
                    for i in range(0, len(resp.plan.poses) - 1)
                ]
            )

            if path_len < min_len:
                min_len = path_len
                waypoint = e

        self.waypoints_history.append(waypoint)
        self.waypoints.remove(waypoint)
        return waypoint

    def send_next_waypoint(self):
        waypoint = self.find_nearest_waypoint()
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
                self.waypoints_history[self.seq],
                a=0.2,
                action=Marker.MODIFY,
            )

            self.seq += 1
            print("Goal reached", res_status)

    def current_pose_callback(self, data):
        self.current_pose = data.pose.pose


if __name__ == "__main__":
    sw = SimpleWaypoints(sys.argv[1])
