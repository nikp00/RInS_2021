import rospy
import math
import tf2_geometry_msgs
import tf2_ros
import tf

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion, Pose, Twist
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.srv import GetPlan
from actionlib_msgs.msg import GoalID


class Mover:
    def __init__(self):
        self.node = rospy.init_node("mover")

        # Parameters
        self.params = {
            "replay_waypoints": rospy.get_param("~replay_waypoints"),
            "rotate_on_replay": rospy.get_param("~rotate_on_replay"),
        }

        # States
        self.states = ["get_next_waypoint", "moving_to_waypoint", "return_home", "end"]
        self.state = {"main": "get_next_waypoint", "rotation": 0, "replay": False}
        self.starting_pose = None

        # Navigation
        self.waypoints = rospy.wait_for_message("/waypoints", PoseArray)
        self.waypoint_markers = MarkerArray()
        self.seq = 0
        self.n_waypoints = len(self.waypoints.poses)
        self.visited_waypoints = list()

        # TF Buffer
        self.tf_buf = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buf)

        # Publishers
        self.waypoint_markers_publisher = rospy.Publisher(
            "/waypoint_markers", MarkerArray, queue_size=10
        )
        self.pose_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )

        # Services
        self.get_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        # Subscribers
        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_sub_callback
        )
        print(self.params)
        self.run()

    def run(self):
        r = rospy.Rate(1)
        while self.pose_publisher.get_num_connections() < 1:
            r.sleep()

        self.starting_pose = self.get_current_pose()

        while not rospy.is_shutdown():
            self.waypoint_markers_publisher.publish(self.waypoint_markers)

            if self.state["main"] == "get_next_waypoint":
                if len(self.waypoints.poses) > 0:
                    nex_waypoint = self.find_nearest_waypoint()
                    self.visited_waypoints.append(nex_waypoint)
                    self.waypoints.poses.remove(nex_waypoint)
                    self.send_next_waypoint(nex_waypoint)
                    self.state["main"] = "moving_to_waypoint"
                    print(self.state)
                elif (
                    self.params["replay_waypoints"] and len(self.visited_waypoints) > 0
                ):
                    next_waypoint = self.visited_waypoints.pop()
                    current_pose = self.get_current_pose()
                    next_waypoint.orientation = self.fix_angle(
                        next_waypoint, current_pose
                    )
                    self.send_next_waypoint(next_waypoint)
                    self.state["main"] = "moving_to_waypoint"
                    self.state["replay"] = True
                    print(self.state)

                else:
                    self.send_next_waypoint(self.starting_pose.pose)
                    self.state["main"] = "return_home"
                    print(self.state)

            elif self.state["main"] == "end":
                break

    def get_current_pose(self) -> PoseStamped:
        pose_translation = None
        while pose_translation is None:
            try:
                pose_translation = self.tf_buf.lookup_transform(
                    "map", "base_link", rospy.Time.now(), rospy.Duration(5)
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

    def find_nearest_waypoint(self, fix_angle=True) -> Pose:
        start = self.get_current_pose()
        min_len = 100000

        # Find the closest waypoint to teh current position
        for e in self.waypoints.poses:
            goal = PoseStamped()
            goal.header.seq = 0
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position = e.position
            goal.pose.orientation = e.orientation

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

        if fix_angle:
            waypoint.orientation = self.fix_angle(waypoint, start)

        return waypoint

    def fix_angle(self, pose: Pose, current_pose: PoseStamped) -> Quaternion:
        dx = pose.position.x - current_pose.pose.position.x
        dy = pose.position.y - current_pose.pose.position.y
        return Quaternion(
            *list(tf.transformations.quaternion_from_euler(0, 0, math.atan2(dy, dx)))
        )

    def send_next_waypoint(self, waypoint: Pose):
        self.waypoint_markers.markers.append(self.create_marker(self.seq, waypoint))

        msg = PoseStamped()
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.position = waypoint.position
        msg.pose.orientation = waypoint.orientation
        self.pose_publisher.publish(msg)

    def create_marker(
        self,
        id,
        waypoint: Pose,
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

        msg.pose.position = waypoint.position
        msg.pose.orientation = Quaternion(0, 0, 0, 1)
        msg.scale.x = sx
        msg.scale.y = sy
        msg.scale.z = 0

        msg.color.r = r
        msg.color.g = g
        msg.color.b = b
        msg.color.a = a

        msg.lifetime = rospy.Duration(lifetime)

        return msg

    def result_sub_callback(self, data):
        res_state = data.status.status
        if self.state["main"] == "moving_to_waypoint":
            if res_state == 3:
                self.seq += 1
                self.state["main"] = "get_next_waypoint"
                self.state["replay"] = False
                print(self.state, 3)
                self.waypoint_markers.markers[-1].color.a = 0.3
            elif res_state == 4:
                self.state["main"] = "get_next_waypoint"
                self.state["replay"] = False
                print(self.state, 4)
        elif self.state["main"] == "return_home":
            if res_state == 3:
                self.state["main"] = "end"
                print(self.state)


if __name__ == "__main__":
    ms = Mover()