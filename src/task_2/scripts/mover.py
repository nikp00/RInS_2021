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
from task_1.srv import TextToSpeechService, TextToSpeechServiceResponse


class Mover:
    def __init__(self):
        self.node = rospy.init_node("mover")
        self.waypoint_markers_publisher = rospy.Publisher(
            "/waypoint_markers", MarkerArray, queue_size=10
        )
        self.face_waypoint_markers_publisher = rospy.Publisher(
            "/face_waypoint_markers", MarkerArray, queue_size=10
        )
        self.pose_publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.cancel_goal_publisher = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=10
        )
        self.rotation_publisher = rospy.Publisher(
            "/cmd_vel_mux/input/teleop", Twist, queue_size=10
        )

        rospy.wait_for_service("text_to_speech")
        self.speak = rospy.ServiceProxy("text_to_speech", TextToSpeechService)

        self.discovered_faces = 0
        self.n_discovered_faces = 0
        self.waypoint_markers = MarkerArray()
        self.face_waypoint_markers = MarkerArray()
        self.replay_waypoints = PoseArray()
        self.seq = 0

        self.states = {
            0: "Get next waypoint",
            1: "Moving to waypoint",
            2: "Move to face",
            3: "Moving to face",
            4: "Speaking to face",
            5: "Return to last saved position",
            6: "Returning to last saved position",
            7: "Rotate",
            8: "Rotating",
            9: "Finished",
            10: "Finished, unsuccessfully",
            11: "End",
        }

        self.rotation_state = -1
        self.state = 0
        self.last_face = list()
        self.stored_pose = None
        self.last_waypoint = None
        self.replay = False
        self.starting_pose = None
        self.number_of_faces = rospy.get_param("~number_of_faces")
        self.distance_to_face = rospy.get_param("~distance_to_face")
        self.enable_rotation = rospy.get_param("~enable_rotation")
        self.waypoints = rospy.wait_for_message("/waypoints", PoseArray)

        self.n_waypoints = len(self.waypoints.poses)

        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_sub_callback
        )

        self.face_sub = rospy.Subscriber(
            "/face_markers", MarkerArray, self.face_markers_sub_callback
        )

        self.get_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        self.tf_buf = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buf)

        self.run()

    def run(self):
        r = rospy.Rate(1)
        while self.pose_publisher.get_num_connections() < 1:
            r.sleep()

        self.starting_pose = self.get_current_pose()

        while not rospy.is_shutdown():
            self.waypoint_markers_publisher.publish(self.waypoint_markers)
            self.face_waypoint_markers_publisher.publish(self.face_waypoint_markers)

            # If there is a new detected face in the queue, move to it
            if self.state in (0, 1) and len(self.last_face) > 0:
                self.state = 2

            # if replay isn't enabled and there is no available waypoint, enable replay
            if not self.replay and len(self.waypoints.poses) == 0:
                self.replay = True
                self.waypoints.poses = self.replay_waypoints.poses[0:-1]
                self.replay_waypoints = None
                if self.enable_rotation:
                    self.speak(
                        "I haven't found all the faces yet but there are no remaining waypoints."
                        + "I will replay all the waypoints in reverse and do a full rotation on every one to maximize my chances to find all the faces."
                    )
                else:
                    self.speak(
                        "I haven't found all the faces yet but there are no remaining waypoints."
                        + "I will replay all the waypoints in reverse to try and find all the faces."
                    )
                print("Replay waypoints in reverse order")

            # State machine
            # state: Get next waypoint
            if self.state == 0:
                # Found all faces
                if self.number_of_faces == self.discovered_faces:
                    self.state = 9
                # Not all waypoints used, send the next nearest waypoint
                elif not self.replay:
                    self.state = 1
                    next_waypoint = self.find_nearest_waypoint()
                    self.last_waypoint = next_waypoint
                    self.replay_waypoints.poses.append(next_waypoint)
                    self.waypoints.poses.remove(next_waypoint)
                    self.send_next_waypoint(next_waypoint)
                    print(self.states[self.state])

                # All waypoints used, all faces not found, replay all the waypoints in the reverse oreder
                elif self.replay and len(self.waypoints.poses) > 0:
                    # Rotation is enabled and robot moved to a new waypoint
                    if self.enable_rotation and self.rotation_state == -1:
                        self.rotation_state = 0
                        self.state = 7
                    else:
                        self.rotation_state = -1
                        print("Remaining waypoints: ", len(self.waypoints.poses))
                        next_waypoint = self.waypoints.poses.pop()
                        self.last_waypoint = next_waypoint
                        current_pose = self.get_current_pose()
                        next_waypoint.orientation = self.fix_angle(
                            next_waypoint, current_pose
                        )
                        self.send_next_waypoint(next_waypoint)
                        self.state = 1
                else:
                    self.state = 10
            # state: Move to face
            elif self.state == 2:
                # Add the current waypoint back to the target list and set state to moving to face
                self.state = 3
                if self.last_waypoint != None:
                    self.waypoints.poses.append(self.last_waypoint)
                self.move_to_face()
                print(self.states[self.state])
            # state: Speaking to face
            elif self.state == 4:
                # Change state to return to last saved point
                self.state = 5
                self.n_discovered_faces += 1
                self.speak(
                    f"Hello, i will call you face number {self.n_discovered_faces}"
                )
                rospy.sleep(3)
                print(self.states[self.state])
            # state: Return to last saved position
            elif self.state == 5:
                # Publish the last stored pose and set state to returning to last saved pose
                self.pose_publisher.publish(self.stored_pose)
                self.state = 6
                print(self.states[self.state])
            elif self.state == 7:
                self.state = 8
                self.rotate()
            # state: Found all faces
            elif self.state == 9:
                self.speak(f"I found all {self.number_of_faces} faces. Returning home")
                self.send_next_waypoint(self.starting_pose.pose)
                self.state = 11
            # state: No waypoints remaining, didn't find all faces"
            elif self.state == 10:
                self.speak(
                    f"I found {self.discovered_faces} out of {self.number_of_faces} faces. Returning home."
                )
                self.send_next_waypoint(self.starting_pose.pose)
                self.state = 11
            # state: Finished
            elif self.state == 11:
                print("Finished")

    def move_to_face(self):
        # Moves robot to the last new detected face

        self.cancel_goal_publisher.publish(GoalID())
        rospy.sleep(1)

        self.stored_pose = self.get_current_pose()

        msg = PoseStamped()
        msg.header.seq = len(self.face_waypoint_markers.markers)
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

        d = 0.05 * self.distance_to_face
        msg.pose.position = Point(
            self.last_face[0].pose.position.x + d * math.cos(angle),
            self.last_face[0].pose.position.y + d * math.sin(angle),
            0,
        )

        # msg.pose.orientation = self.stored_pose.pose.orientation
        msg.pose.orientation = self.fix_angle(self.last_face[0].pose, msg)

        self.face_waypoint_markers.markers.append(
            self.create_marker(len(self.face_waypoint_markers.markers), msg.pose, b=255)
        )

        self.last_face = self.last_face[1:]

        self.pose_publisher.publish(msg)

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
        # Calculates the angle at which the robot saw the face
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

    def rotate(self):
        if self.rotation_state == 0:
            self.stored_pose = self.get_current_pose()
            print("Stored pose")

        self.rotation_state += 1
        c_pose = self.get_current_pose()
        rz = tf.transformations.euler_from_quaternion(
            [
                c_pose.pose.orientation.x,
                c_pose.pose.orientation.y,
                c_pose.pose.orientation.z,
                c_pose.pose.orientation.w,
            ]
        )[2]
        rz += math.pi / 2
        c_pose.pose.orientation = Quaternion(
            *list(tf.transformations.quaternion_from_euler(0, 0, rz))
        )
        print(self.rotation_state, rz)
        self.pose_publisher.publish(c_pose)

    def result_sub_callback(self, data):

        # State machine
        # state: Moving to waypoint
        if self.state == 1:
            if data.status.status == 3:
                """Waypoint reached successfully, increment seq and
                set state to get next waypoint"""
                self.seq += 1
                self.state = 0
                print(self.states[self.state])
            elif data.status.status == 4:
                self.seq += 1
                self.state = 0
                print("Goal cant be reached")
        # state: Moving to face
        elif self.state == 3:
            if data.status.status == 3:
                """Face reached successfully, change state to Speaking to face"""
                self.state = 4
                print(self.states[self.state])
            elif data.status.status == 4:
                """Face couldn't be reached, change state to Return to last saved position"""
                self.state = 5
        # Returning to last saved position
        elif self.state == 6:
            if data.status.status == 3:
                """Saved position reached successfully, change state to Get next waypoint"""
                self.state = 0
                print(self.states[self.state])
        elif self.state == 8:
            if data.status.status == 3:
                if self.rotation_state < 3:
                    self.state = 7
                    print("Next rotation")
                else:
                    print("Resume movement")
                    self.state = 0
                    self.rotation_state = 0
            else:
                print("Resume movement")
                self.state = 0
                self.rotation_state = 0

    def face_markers_sub_callback(self, data):
        if self.discovered_faces < len(data.markers):
            self.discovered_faces = len(data.markers)
            self.last_face.append(data.markers[-1])


if __name__ == "__main__":
    ms = Mover()