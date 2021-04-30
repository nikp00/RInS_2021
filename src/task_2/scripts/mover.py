import rospy
import math
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import tf
import cv2
from cv_bridge import CvBridge, CvBridgeError

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion, Pose, Twist
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.srv import GetPlan
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from task_2.msg import PoseAndColorArray


class Mover:
    def __init__(self):
        self.node = rospy.init_node("mover")
        self.bridge = CvBridge()

        self.forward_counter = 0
        self.back_counter = 0
        self.move_forward = False
        self.last_turn = None

        # Parameters
        self.params = {
            "replay_waypoints": rospy.get_param("~replay_waypoints", default=True),
            "rotate_on_replay": rospy.get_param("~rotate_on_replay", default=False),
            "distance_to_cylinder": rospy.get_param(
                "~distance_to_cylinder", default=10
            ),
        }

        # Detected objects data
        self.rings = {"data": PoseArray(), "last_id": 0}
        self.cylinders = {
            "data": PoseAndColorArray(),
            "last_id": 0,
            "cancel_counter": 0,
        }

        # Other data
        self.starting_pose = None
        self.stored_pose = None
        self.image = None

        # States
        self.states = [
            "get_next_waypoint",
            "moving_to_waypoint",
            "move_to_cylinder",
            "moving_to_cylinder",
            "move_to_ring",
            "moving_to_ring",
            "return_to_stored_pose",
            "return_home",
            "end",
        ]
        self.state = {
            "main": "get_next_waypoint",
            "rotation": 0,
            "replay": False,
        }

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
        self.waypoint_markers_pub = rospy.Publisher(
            "/waypoint_markers", MarkerArray, queue_size=10
        )
        self.pose_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )
        self.cancel_goal_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=10
        )
        self.fine_navigation_img_pub = rospy.Publisher(
            "/fine_navigation_image", Image, queue_size=10
        )
        self.twist_pub = rospy.Publisher(
            "/cmd_vel_mux/input/teleop", Twist, queue_size=10
        )
        self.arm_control_pub = rospy.Publisher("/arm_command", String, queue_size=10)

        # Services
        self.get_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        # Subscribers
        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_sub_callback
        )
        self.cylinder_sub = rospy.Subscriber(
            "/cylinder_pose", PoseAndColorArray, self.cylinder_sub_callback
        )
        # self.image_sub = rospy.Subscriber(
        #     "/camera/rgb/image_raw", Image, self.image_callback
        # )
        print(self.params)
        self.run()

    def run(self):
        r = rospy.Rate(1)
        while self.pose_pub.get_num_connections() < 1:
            r.sleep()

        self.starting_pose = self.get_current_pose()

        while not rospy.is_shutdown():
            r.sleep()
            self.waypoint_markers_pub.publish(self.waypoint_markers)

            if (
                self.state["main"] in ("get_next_waypoint", "moving_to_waypoint")
                and len(self.cylinders["data"].cylinders) > self.cylinders["last_id"]
            ):
                self.state["main"] = "move_to_cylinder"
            elif (
                self.state["main"] in ("get_next_waypoint", "moving_to_waypoint")
                and len(self.cylinders["data"].cylinders) > self.cylinders["last_id"]
            ):
                self.state["main"] = "move_to_ring"

            if self.state["main"] == "get_next_waypoint":
                if len(self.waypoints.poses) > 0:
                    next_waypoint = self.find_nearest_waypoint()
                    self.visited_waypoints.append(next_waypoint)
                    self.waypoints.poses.remove(next_waypoint)
                    self.waypoint_markers.markers.append(
                        self.create_marker(self.seq, next_waypoint)
                    )
                    self.send_next_waypoint(next_waypoint)
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
            elif self.state["main"] == "move_to_cylinder":
                self.cancel_goal_pub.publish(GoalID())
                rospy.sleep(1)
                self.stored_pose = self.get_current_pose()
                if len(self.visited_waypoints) > 0:
                    last_waypoint = self.visited_waypoints.pop()
                    self.waypoints.poses.append(last_waypoint)
                if len(self.waypoint_markers.markers) > 0:
                    self.waypoint_markers.markers.pop()
                cylinder = (
                    self.cylinders["data"].cylinders[self.cylinders["last_id"]].pose
                )
                self.cylinders["last_id"] += 1
                self.move_to_cylinder(cylinder, self.params["distance_to_cylinder"])
                self.state["main"] = "moving_to_cylinder"
                print(self.state)
            elif self.state["main"] == "approach_cylinder":
                self.approach_cylinder()
            elif self.state["main"] == "return_to_stored_pose":
                self.pose_pub.publish(self.stored_pose)
                self.stored_pose = None
                self.state["main"] = "moving_to_waypoint"
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
        msg = PoseStamped()
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.position = waypoint.position
        msg.pose.orientation = waypoint.orientation
        self.pose_pub.publish(msg)

    def move_to_cylinder(self, cylinder: Pose, distance_to_cylinder: int):
        msg = PoseStamped()
        msg.header.seq = len(self.cylinders["data"].cylinders)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        angle = tf.transformations.euler_from_quaternion(
            [
                cylinder.orientation.x,
                cylinder.orientation.y,
                cylinder.orientation.z,
                cylinder.orientation.w,
            ]
        )[2]

        d = 0.05 * distance_to_cylinder

        msg.pose.position = Point(
            cylinder.position.x + d * math.cos(angle),
            cylinder.position.y + d * math.sin(angle),
            0,
        )

        msg.pose.orientation = self.fix_angle(cylinder, msg)

        self.waypoint_markers.markers.append(
            self.create_marker(self.seq + 100, msg.pose, b=1)
        )

        self.pose_pub.publish(msg)

    def approach_cylinder(self):
        color = self.cylinders["data"].cylinders[self.cylinders["last_id"] - 1].color
        try:
            image = self.bridge.imgmsg_to_cv2(
                rospy.wait_for_message("/camera/rgb/image_raw", Image), "bgr8"
            )
        except CvBridgeError as e:
            print(e)

        limits = {
            "red": (np.array([0, 10, 65]), np.array([20, 255, 255])),
            "yellow": (np.array([20, 10, 65]), np.array([40, 255, 255])),
            "green": (np.array([40, 10, 65]), np.array([70, 255, 255])),
            "blue": (np.array([70, 10, 65]), np.array([110, 255, 255])),
        }

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv, *limits.get(color, (np.array([0, 0, 0]), np.array([110, 255, 255])))
        )
        countour, hierarchy = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        left_right = 100
        top_bottom = 170

        if len(countour) > 0:
            depth_image = rospy.wait_for_message("/camera/depth/image_raw", Image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")

            temp = cv2.bitwise_and(depth_image, depth_image, mask=cv2.bitwise_not(mask))
            left = np.nanmean(temp[top_bottom:-top_bottom, 0:left_right])
            right = np.nanmean(
                temp[top_bottom:-top_bottom, depth_image.shape[1] - left_right :]
            )

            yd = int(depth_image.shape[0] / 2)
            xd = int(depth_image.shape[1] / 2)
            dist = np.nanmean(depth_image[yd - 5 : yd + 5, xd - 5 : xd + 5])
            print()

            cont = max(countour, key=cv2.contourArea)
            m = cv2.moments(cont)
            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])
            c_dist = depth_image[cy, cx]
            msg = Twist()

            print("LEft: ", left, ", Right: ", right, ", C_dist: ", c_dist)
            if (np.isnan(left) or left < 0.5) and c_dist > 0.4:
                msg.angular.z = -0.2
                print("Avoid, turn right")
                self.last_turn = "right"
                self.move_forward = 5
            elif (np.isnan(right) or right < 0.5) and c_dist > 0.4:
                print("Avoid, turn left")
                msg.angular.z = 0.2
                self.last_turn = "left"
                self.move_forward = 5
            elif self.move_forward > 0:
                print("Avoid, forward")
                self.move_forward -= 1
                if c_dist > 0.5:
                    msg.linear.x = 0.05
                else:
                    self.move_forward = 0
            elif self.last_turn != None:
                self.last_turn = None
                print("Avoid, rotate back")
                if c_dist > 0.5:
                    if self.last_turn == "left":
                        msg.angular.z = -0.5
                    else:
                        msg.angular.z = 0.5
            else:

                cv2.circle(image, (cx, cy), 3, (0, 255, 0))
                cv2.drawContours(image, [cont], -1, (0, 255, 0), 2)

                y = int(image.shape[0] / 2)
                x = int(image.shape[1] / 2)

                step = 0.08
                if abs(cx - x) < 10:
                    step = 0.005
                elif abs(cx - x) < 30:
                    step = 0.01
                elif abs(cx - x) < 50:
                    step = 0.03

                if self.back_counter == 0:
                    if abs(cx - x) < 2:
                        if not np.isnan(dist):
                            if dist > 0.46:
                                msg.linear.x = 0.05
                            else:
                                msg.linear.x = 0.01
                            print("forward")
                        elif self.forward_counter < 12:
                            msg.linear.x = 0.01
                            self.forward_counter += 1
                            print("forward manual", self.forward_counter)
                        elif self.forward_counter < 17:
                            msg.linear.x = 0.005
                            self.forward_counter += 1
                            print("forward manual", self.forward_counter)
                        else:
                            if self.back_counter == 0:
                                self.arm_control_pub.publish("extend")
                                rospy.sleep(5)
                                self.arm_control_pub.publish("retract")
                                self.back_counter = 1
                                print("Move arm")
                    else:
                        if cx > x:
                            print("Turn right", cx, x, step)
                            msg.angular.z = -step
                        elif cy < x:
                            print("Turn left", cx, x, step)
                            msg.angular.z = step
                else:
                    if self.back_counter < 4:
                        msg.linear.x = -0.2
                        self.back_counter += 1
                        print("Back", self.back_counter)
                    else:
                        self.back_counter = 0
                        self.forward_counter = 0
                        self.state["main"] = "return_to_stored_pose"
                        print("Return")

            self.twist_pub.publish(msg)
        image[top_bottom:-top_bottom, 0:left_right] = (0, 0, 255)
        image[top_bottom:-top_bottom, image.shape[1] - left_right :] = (0, 0, 255)
        self.fine_navigation_img_pub.publish(
            self.bridge.cv2_to_imgmsg(
                cv2.bitwise_and(image, image, mask=cv2.bitwise_not(mask))
            )
        )

    def fade_markers(self):
        for e in self.waypoint_markers.markers:
            e.color.a = 0.3

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

    # def image_callback(self, data):
    #     try:
    #         self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         print(e)

    #     limits = {
    #         "red": (np.array([0, 10, 65]), np.array([20, 255, 255])),
    #         "yellow": (np.array([20, 10, 65]), np.array([40, 255, 255])),
    #         "green": (np.array([40, 10, 65]), np.array([70, 255, 255])),
    #         "blue": (np.array([70, 10, 65]), np.array([110, 255, 255])),
    #     }

    #     hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    #     mask = cv2.inRange(hsv, *limits["red"])
    #     countour, hierarchy = cv2.findContours(
    #         mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    #     )
    #     if len(countour) > 0:
    #         depth_image = rospy.wait_for_message("/camera/depth/image_raw", Image)
    #         depth_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")
    #         yd = int(depth_image.shape[0] / 2)
    #         xd = int(depth_image.shape[1] / 2)
    #         dist = np.nanmean(depth_image[yd - 5 : yd + 5, xd - 5 : xd + 5])
    #         print()
    #         print(dist)

    #         cont = max(countour, key=cv2.contourArea)
    #         m = cv2.moments(cont)
    #         cx = int(m["m10"] / m["m00"])
    #         cy = int(m["m01"] / m["m00"])
    #         cv2.circle(self.image, (cx, cy), 3, (0, 255, 0))
    #         cv2.drawContours(self.image, [cont], -1, (0, 255, 0), 2)

    #         msg = Twist()

    #         y = int(self.image.shape[0] / 2)
    #         x = int(self.image.shape[1] / 2)

    #         step = 0.08
    #         # if abs(cx - x) > 100:
    #         # step = 0.08
    #         if abs(cx - x) < 10:
    #             step = 0.005
    #         elif abs(cx - x) < 30:
    #             step = 0.01
    #         elif abs(cx - x) < 50:
    #             step = 0.03
    #         if abs(cx - x) < 2:
    #             if not np.isnan(dist):
    #                 msg.linear.x = 0.05
    #                 print("forward")
    #             elif self.forward_counter < 40:
    #                 msg.linear.x = 0.05
    #                 self.forward_counter += 1
    #                 print("forward manual", self.forward_counter)
    #             else:
    #                 print("center")

    #         else:
    #             if cx > x:
    #                 print("Turn right", cx, x, step)
    #                 msg.angular.z = -step
    #             elif cy < x:
    #                 print("Turn left", cx, x, step)
    #                 msg.angular.z = step

    #         self.twist_pub.publish(msg)
    # self.fine_navigation_img_pub.publish(self.bridge.cv2_to_imgmsg(self.image))

    def result_sub_callback(self, data):
        res_state = data.status.status
        if self.state["main"] == "moving_to_waypoint":
            if res_state in (3, 4):
                self.seq += 1
                self.state["main"] = "get_next_waypoint"
                self.state["replay"] = False
                print(self.state, 3)
                self.fade_markers()
        elif self.state["main"] == "moving_to_cylinder":
            if res_state == 3:
                self.state["main"] = "approach_cylinder"
                # self.state["main"] = "return_to_stored_pose"
            if res_state == 4:
                if self.cylinders["cancel_counter"] < 2:
                    self.cylinders["cancel_counter"] += 1
                    cylinder = (
                        self.cylinders["data"]
                        .cylinders[self.cylinders["last_id"] - 1]
                        .pose
                    )
                    self.move_to_cylinder(
                        cylinder,
                        self.params["distance_to_cylinder"]
                        + self.cylinders["cancel_counter"] * 5,
                    )
                    self.waypoint_markers.markers.pop()
                else:
                    self.cylinders["cancel_counter"] = 0
                    self.state["main"] = "return_to_stored_pose"
        elif self.state["main"] == "return_home":
            if res_state == 3:
                self.state["main"] = "end"
                print(self.state)

    def cylinder_sub_callback(self, data):
        self.cylinders["data"] = data


if __name__ == "__main__":
    ms = Mover()
