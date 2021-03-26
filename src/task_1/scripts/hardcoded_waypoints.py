import rospy
import json
import sys
import tf

from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion


class WaypointReader:
    def __init__(self, waypoints_file):
        self.node = rospy.init_node("hardcoded_waypoints")
        self.waypoint_publisher = rospy.Publisher(
            "/waypoints", PoseArray, queue_size=10
        )
        self.waypoints = json.load(open(waypoints_file))
        self.n_connections = 0

    def publish(self):
        n = self.waypoint_publisher.get_num_connections()
        if n > self.n_connections:
            self.n_connections = n
            self.waypoint_publisher.publish(self.parsed_waypoints)
        else:
            self.n_connections = n

    def parse_waypoints(self):
        self.parsed_waypoints = PoseArray()

        for e in self.waypoints:
            pose = Pose()
            pose.position = Point(*e["position"])
            pose.orientation = Quaternion(
                *list(
                    tf.transformations.quaternion_from_euler(0, 0, e["orientation"][2])
                )
            )
            self.parsed_waypoints.poses.append(pose)


if __name__ == "__main__":
    wr = WaypointReader(sys.argv[1])
    wr.parse_waypoints()

    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        wr.publish()
        r.sleep()
