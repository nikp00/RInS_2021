import rospy
import math
import tf2_geometry_msgs
import tf2_ros
import tf

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.srv import GetPlan
from actionlib_msgs.msg import GoalID


class Mover:
    def __init__(self):
        self.node = rospy.init_node("mover")
        self.waypoint_markers_publisher = rospy.Publisher(
            "/waypoint_markers", MarkerArray, queue_size=10
        )
        self.pose_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.cancel_goal_publisher = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=10
        )

        self.discovered_faces = 0
        self.waypoint_markers = MarkerArray()
        self.seq = 0
        self.states = {
            0: "Get next waypoint",
            1: "Moving to waypoint",
            2: "Move to face",
            3: "Moving to face",
            4: "Speaking to face",
            5: "Return to last saved position",
            6: "Returning to last saved position",
        }
        self.state = 0
        self.last_face = list()
        self.stored_pose = None
        self.last_waypoint = None

        self.waypoints = rospy.wait_for_message("/waypoints", PoseArray)
        self.n_waypoints = len(self.waypoints.poses)

        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_sub_callback
        )

        self.face_sub = rospy.Subscriber(
            "/face_markers", MarkerArray, self.face_markers_sub_callbask
        )

        self.get_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        self.tf_buf = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buf)

        self.run()

    def run(self):
        r = rospy.Rate(1)
        while self.pose_publisher.get_num_connections() < 1:
            r.sleep()

        while not rospy.is_shutdown():
            if self.state not in (2, 3, 4, 5, 6) and len(self.last_face) > 0:
                self.state = 2

            self.waypoint_markers_publisher.publish(self.waypoint_markers)
            if self.seq < self.n_waypoints:
                if self.state == 0:
                    self.state = 1
                    print(self.states[self.state])
                    self.send_next_waypoint()
                if self.state == 2:
                    self.state = 3
                    print(self.states[self.state])
                    self.waypoints.poses.append(self.last_waypoint)
                    self.move_to_face()
                if self.state == 4:
                    self.state = 5
                    print(self.states[self.state])
                if self.state == 5:
                    self.pose_publisher.publish(self.stored_pose)
                    self.state = 6
                    print(self.states[self.state])

    def move_to_face(self):
        self.cancel_goal_publisher.publish(GoalID())
        self.stored_pose = self.get_current_pose()

        msg = PoseStamped()
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        angle = tf.transformations.euler_from_quaternion(
            [
                self.last_face[0].pose.orientation.x,
                self.last_face[0].pose.orientation.y,
                self.last_face[0].pose.orientation.z,
                self.last_face[0].pose.orientation.w,
            ]
        )[2]

        d = 0.05 * 10
        msg.pose.position = Point(
            self.last_face[0].pose.position.x + d * math.cos(angle),
            self.last_face[0].pose.position.y + d * math.sin(angle),
            0,
        )
        msg.pose.orientation = self.stored_pose.pose.orientation

        self.waypoint_markers.markers.append(
            self.create_marker(self.seq + 95, msg.pose, b=255)
        )

        self.last_face = self.last_face[1:]

        self.pose_publisher.publish(msg)

    def get_current_pose(self):
        pose_translation = None
        while pose_translation is None:
            try:
                pose_translation = self.tf_buf.lookup_transform(
                    "map", "base_link", rospy.Time.now(), rospy.Duration(2)
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

    def find_nearest_waypoint(self):
        start = self.get_current_pose()
        min_len = 100000

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

        if True:
            waypoint.orientation = self.fix_angle(waypoint, start)

        self.last_waypoint = waypoint

        self.waypoints.poses.remove(waypoint)
        return waypoint

    def fix_angle(self, pose, current_pose):
        dx = pose.position.x - current_pose.pose.position.x
        dy = pose.position.y - current_pose.pose.position.y
        return Quaternion(
            *list(tf.transformations.quaternion_from_euler(0, 0, math.atan2(dy, dx)))
        )

    def send_next_waypoint(self):
        waypoint = self.find_nearest_waypoint()
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
        if self.state == 1:  # Moving to waypoint
            if data.status.status == 3:
                self.seq += 1
                self.state = 0
                print(self.states[self.state])
        elif self.state == 3:  # Moving to face
            if data.status.status == 3:
                self.state = 4
                print(self.states[self.state])
        elif self.state == 6:  # Moving to stored pose
            if data.status.status == 3:
                self.state = 0
                print(self.states[self.state])

    def face_markers_sub_callbask(self, data):
        if self.discovered_faces < len(data.markers):
            self.discovered_faces = len(data.markers)
            self.last_face.append(data.markers[-1])


if __name__ == "__main__":
    ms = Mover()