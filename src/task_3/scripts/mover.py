from cv2 import data
import rospy
import math
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import tf
import cv2
from cv_bridge import CvBridge, CvBridgeError
import copy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Point,
    Quaternion,
    Pose,
    Twist,
)
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.srv import GetPlan
from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import Image
from std_msgs.msg import String
from task_3.msg import PoseAndColorArray, FaceDataArray
from task_3.srv import TextToSpeechService, QRCodeReaderService, ExtractDigitsService, DialogService
from kobuki_msgs.msg import BumperEvent

import requests
import pandas as pd
import io

from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score


class Mover:
    def __init__(self):
        self.node = rospy.init_node("mover")
        self.bridge = CvBridge()

        # Ring and cylinder approach
        self.forward_counter = 0
        self.back_counter = None
        self.move_forward = False
        self.last_turn = None
        self.goal_reached = False

        # Ring approach
        self.last_distance = None
        self.ring_default_distance = 0.09
        self.ring_target_distance = 0.09
        self.bumper_hit = False
        self.bumper_hit_counter = 0

        # Parameters
        self.params = {
            "distance_to_cylinder": rospy.get_param("~distance_to_cylinder", default=7),
            "distance_to_ring": rospy.get_param("~distance_to_ring", default=10),
            "horizontal_space": rospy.get_param("~horizontal_space", default=100),
            "vertical_space": rospy.get_param("~vertical_space", default=170),
            "cylinder_detection_min_dis": rospy.get_param(
                "~cylinder_detection_min_dis", default=0.6
            ),
        }

        # Detected objects data
        self.rings = ObjectContainer()
        self.cylinders = ObjectContainer()
        self.faces = ObjectContainer()

        # Other data
        self.starting_pose = None
        self.stored_pose = None
        self.image = None

        # Face approach data
        self.updated_pose = False
        self.turn_counter = 0

        # States
        self.states = [
            "get_next_waypoint",
            "moving_to_waypoint",
            "move_to_cylinder",
            "find_qr_code",
            "moving_to_cylinder",
            "move_to_face",
            "moving_to_face",
            "move_to_ring",
            "moving_to_ring",
            "return_to_stored_pose",
            "return_home",
            "end",
        ]
        self.state = "get_next_waypoint"

        self.searching_cylinder = False
        self.searching_ring = False
        self.cylinder_found = False
        self.ring_found = False
        self.delivering_vaccine = False

        # Navigation
        self.waypoints = Waypoints()
        self.seq = 0
        self.waypoint_markers = MarkerArray()
        self.vaccinated_faces = 0
        self.get_waypoints()

        # TF Buffer
        self.tf_buf = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf_buf)

        # Publishers
        self.waypoint_markers_pub = rospy.Publisher("/waypoint_markers", MarkerArray, queue_size=10)
        self.pose_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.cancel_goal_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        self.fine_navigation_img_pub = rospy.Publisher(
            "/fine_navigation_image", Image, queue_size=10
        )
        self.twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.arm_control_pub = rospy.Publisher("/arm_command", String, queue_size=10)

        # Services
        self.get_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        rospy.wait_for_service("text_to_speech")
        self.speak = rospy.ServiceProxy("text_to_speech", TextToSpeechService)
        rospy.wait_for_service("qr_code_reader")
        self.extract_qr_code = rospy.ServiceProxy("qr_code_reader", QRCodeReaderService)
        rospy.wait_for_service("digit_extractor")
        self.extract_digits = rospy.ServiceProxy("digit_extractor", ExtractDigitsService)
        rospy.wait_for_service("dialog_service")
        self.dialog = rospy.ServiceProxy("dialog_service", DialogService)

        # Subscribers
        self.result_sub = rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.result_sub_callback
        )
        self.cylinder_sub = rospy.Subscriber(
            "/cylinder_pose", PoseAndColorArray, self.cylinder_sub_callback
        )
        self.ring_sub = rospy.Subscriber("/ring_pose", PoseAndColorArray, self.ring_sub_callback)
        self.face_sub = rospy.Subscriber("/face_pose", FaceDataArray, self.face_sub_callback)
        self.bumper_sub = rospy.Subscriber(
            "/mobile_base/events/bumper", BumperEvent, self.bumper_callback
        )

        print(self.params)
        rospy.sleep(5)
        self.run()

    def run(self):
        r = rospy.Rate(1)
        while self.pose_pub.get_num_connections() < 1:
            r.sleep()

        self.starting_pose = self.get_current_pose()

        while not rospy.is_shutdown():
            r.sleep()
            self.waypoint_markers_pub.publish(self.waypoint_markers)

            if self.state in (
                "get_next_waypoint",
                "moving_to_waypoint",
                "return_home",
                "end",
            ):
                self.check_social_distance()

            if self.state in (
                "get_next_waypoint",
                "moving_to_waypoint",
                "return_home",
                "end",
            ):
                if self.searching_ring and self.ring_found:
                    self.state = "proccess_face"
                elif self.cylinders.is_new_data() and not any(
                    [self.searching_ring, self.delivering_vaccine]
                ):
                    self.state = "move_to_cylinder"
                elif self.faces.is_new_data() and not any(
                    [self.searching_cylinder, self.searching_ring, self.delivering_vaccine]
                ):
                    self.state = "move_to_face"

            if self.vaccinated_faces == 4:
                self.send_next_waypoint(self.starting_pose.pose)
                self.state = "return_home"

            elif self.state == "get_next_waypoint":

                if len(self.waypoints.get_available()) > 0:
                    next_waypoint = self.find_nearest_waypoint()
                    self.waypoint_markers.markers.append(
                        self.create_marker(self.seq, next_waypoint.pose)
                    )
                    self.send_next_waypoint(next_waypoint.pose)
                    self.state = "moving_to_waypoint"
                elif len(self.waypoints.history) > 0:
                    next_waypoint = self.waypoints.history.pop()
                    current_pose = self.get_current_pose()
                    next_waypoint.pose.orientation = self.fix_angle(
                        next_waypoint.pose, current_pose.pose
                    )
                    self.send_next_waypoint(next_waypoint.pose)
                    self.state = "moving_to_waypoint"
                else:
                    self.waypoints.reset()

            elif self.state == "move_to_cylinder":
                self.interupt_plan()
                cylinder = self.cylinders.get_last()
                self.move_to_cylinder(cylinder)
                self.state = "moving_to_cylinder"
            elif self.state == "read_qr_code":
                self.read_qr_code()

            elif self.state == "move_to_ring":
                self.interupt_plan()
                ring = self.rings.current
                self.move_to_ring(ring)
                self.state = "moving_to_ring"
            elif self.state == "approach_ring":
                self.approach_ring()

            elif self.state == "move_to_face":
                self.interupt_plan()
                face = self.faces.get_last()
                self.move_to_face(face)
                self.state = "moving_to_face"
            elif self.state == "initial_dialog":
                self.get_face_info()
            elif self.state == "read_digits":
                print("Read digits")
                self.read_digits()

            elif self.state == "warning_safety_distance":
                print("WARNING")
                self.speak("Please respect the safety distance!")
                self.state = "return_to_stored_pose"

            elif self.state == "proccess_face":
                self.searching_cylinder = False
                self.searching_ring = False
                if self.faces.current == None or self.faces.current.proccessed:
                    self.state = "return_to_stored_pose"
                else:
                    if not self.faces.current.already_vaccinated:
                        cylinder = self.cylinders.get_by(self.faces.current.doctor, "color")
                        print("Cylinder", self.faces.current.doctor)
                        if cylinder != None and cylinder.clf != None:
                            vaccine = cylinder.clf.predict(
                                self.faces.current.age, self.faces.current.hours_of_exercise
                            )
                            print("Vaccine", vaccine, "Cylinder", cylinder.color)
                            self.faces.current.vaccine = vaccine
                            ring = self.rings.get_by(vaccine, "vaccine")

                            if ring != None:
                                self.rings.current = ring
                                self.delivering_vaccine = True
                                self.state = "moving_to_ring"
                                self.move_to_ring(ring)
                            else:
                                self.searching_ring = True
                                self.state = "return_to_stored_pose"
                        else:
                            self.searching_cylinder = True
                            self.state = "return_to_stored_pose"
                    else:
                        self.vaccinated_faces += 1
                        self.faces.current.proccessed = True
                        self.state = "return_to_stored_pose"

            elif self.state == "give_vaccine":
                self.speak(f"Here is your {self.faces.current.vaccine} vaccine")
                self.arm_control_pub.publish("extend")
                rospy.sleep(3)
                self.arm_control_pub.publish("retract")
                self.delivering_vaccine = False
                self.searching_cylinder = False
                self.searching_ring = False
                self.faces.current.already_vaccinated = True
                self.faces.current.proccessed = True
                self.vaccinated_faces += 1
                self.state = "get_next_waypoint"

            elif self.state == "return_to_stored_pose":
                if self.stored_pose != None:
                    self.pose_pub.publish(self.stored_pose)
                    self.stored_pose = None
                    self.state = "moving_to_waypoint"
                else:
                    self.state = "get_next_waypoint"
            elif self.state == "end":
                break

            print(self.state, self.delivering_vaccine, self.vaccinated_faces)

    def get_waypoints(self):
        waypoints = rospy.wait_for_message("/waypoints", PoseArray)
        for e in waypoints.poses:
            self.waypoints.add(Waypoint(e))

    def interupt_plan(self):
        self.cancel_goal_pub.publish(GoalID())
        rospy.sleep(1)
        self.stored_pose = self.get_current_pose()
        self.fade_markers()

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
        for e in self.waypoints.get_available():
            goal = PoseStamped()
            goal.header.seq = 0
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position = e.pose.position
            goal.pose.orientation = e.pose.orientation

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
            waypoint.pose.orientation = self.fix_angle(waypoint.pose, start.pose)

        self.waypoints.set_current(waypoint)
        return waypoint

    def fix_angle(self, pose: Pose, current_pose: Pose) -> Quaternion:
        dx = pose.position.x - current_pose.position.x
        dy = pose.position.y - current_pose.position.y
        return Quaternion(*list(tf.transformations.quaternion_from_euler(0, 0, math.atan2(dy, dx))))

    def send_next_waypoint(self, waypoint: Pose):
        msg = PoseStamped()
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.position = waypoint.position
        msg.pose.orientation = waypoint.orientation
        self.pose_pub.publish(msg)

    def move_to_cylinder(self, cylinder):
        msg = PoseStamped()
        msg.header.seq = self.cylinders.current.id
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose = cylinder.navigation_pose

        self.pose_pub.publish(msg)
        self.waypoint_markers.markers.append(self.create_marker(self.seq + 100, msg.pose, b=1))

    def read_qr_code(self):
        res = self.extract_qr_code(0)
        if res.status == 0:
            print("Detected QR code:", res.data)
            self.cylinders.current.add_clf(str(res.data))

            self.speak(
                f"I just consulted with doctor {self.cylinders.current.color} about which vaccine is the best choice for his patients."
            )

            msg = Twist()
            msg.linear.x = -0.5
            self.twist_pub.publish(msg)
            rospy.sleep(2)

            # if (
            #     self.searching_cylinder
            #     and self.faces.current.doctor == self.cylinders.current.color
            # ):
            #     self.state = "proccess_face"
            # else:
            #     self.state = "return_to_stored_pose"

            self.state = "proccess_face"
        else:
            print("No QR code found, fixing orientation")
            msg = Twist()
            msg.linear.x = 0.1
            self.twist_pub.publish(msg)
            rospy.sleep(1)

    def approach_cylinder(self, cylinder):
        color = cylinder.color

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
            hsv,
            *limits.get(color, (np.array([0, 0, 0]), np.array([110, 255, 255]))),
        )
        countour, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(countour) > 0:
            depth_image = rospy.wait_for_message("/camera/depth/image_raw", Image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")

            y = int(image.shape[0] / 2)
            x = int(image.shape[1] / 2)

            yd = int(depth_image.shape[0] / 2)
            xd = int(depth_image.shape[1] / 2)
            dist = np.nanmean(depth_image[yd - 5 : yd + 5, xd - 5 : xd + 5])

            cont = max(countour, key=cv2.contourArea)
            m = cv2.moments(cont)
            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])

            c_dist = depth_image[cy, cx]

            cv2.circle(image, (cx, cy), 3, (0, 255, 0))
            cv2.drawContours(image, [cont], -1, (0, 255, 0), 2)

            image[
                self.params["vertical_space"] : -self.params["vertical_space"],
                0 : self.params["horizontal_space"],
            ] = (0, 0, 255)

            image[
                self.params["vertical_space"] : -self.params["vertical_space"],
                image.shape[1] - self.params["horizontal_space"] :,
            ] = (0, 0, 255)

            # Compute the distance to the left and right side while ignoring the detected cylinder
            temp_left = list()
            for i in range(
                self.params["vertical_space"],
                depth_image.shape[0] - self.params["vertical_space"],
            ):
                for j in range(self.params["horizontal_space"]):
                    if mask[i, j] != 255:
                        temp_left.append(depth_image[i, j])
            temp_left = np.array(temp_left)

            temp_right = list()
            for i in range(
                self.params["vertical_space"],
                depth_image.shape[0] - self.params["vertical_space"],
            ):
                for j in range(
                    depth_image.shape[1] - self.params["horizontal_space"],
                    depth_image.shape[1],
                ):
                    if mask[i, j] != 255:
                        temp_right.append(depth_image[i, j])
            temp_right = np.array(temp_right)

            left = np.nanmean(temp_left)
            right = np.nanmean(temp_right)

            msg = Twist()

            print(
                f"Distance to image center: {dist:.4}, Distance to cylinder center: {c_dist:.4}, image center: {x}, cylinder center: {cx}, left: {left:.4}, right: {right:.4}"
            )

            if not self.goal_reached:
                if self.last_turn != None and cy < 100:
                    if self.last_turn == "left":
                        msg.angular.z = -0.5
                    else:
                        msg.angular.z = 0.5

                if (np.isnan(left) or left < 0.5) and c_dist > self.params[
                    "cylinder_detection_min_dis"
                ]:
                    msg.angular.z = -0.5
                    self.last_turn = "right"
                    self.move_forward = 1
                    print("Avoid, turn right (speed: 0.5)")
                elif (np.isnan(right) or right < 0.5) and c_dist > self.params[
                    "cylinder_detection_min_dis"
                ]:
                    msg.angular.z = 0.5
                    self.last_turn = "left"
                    self.move_forward = 1
                    print("Avoid, turn left (speed: 0.5)")
                elif self.move_forward > 0:
                    self.move_forward -= 1
                    if c_dist > self.params["cylinder_detection_min_dis"]:
                        msg.linear.x = 0.2
                        print("Avoid, forward (speed: 0.2)")
                    elif c_dist > 0.5:
                        msg.linear.x = 0.05
                        print("Avoid, forward (speed: 0.5)")
                    elif c_dist > 0.4:
                        msg.linear.x = 0.01
                        print("Avoid, forward (speed: 0.01)")
                    else:
                        self.move_forward = 0
                elif self.last_turn != None:
                    self.last_turn = None
                    if c_dist > self.params["cylinder_detection_min_dis"]:
                        if self.last_turn == "left":
                            msg.angular.z = -0.5
                            print("Avoid, rotate back right (speed: 0.5)")
                        else:
                            msg.angular.z = 0.5
                            print("Avoid, rotate back left (speed: 0.5)")
                else:
                    # Set rotation step
                    step = 0.15
                    if abs(cx - x) < 5:
                        step = 0.005
                    elif abs(cx - x) < 20:
                        step = 0.01
                    elif abs(cx - x) < 30:
                        step = 0.03
                    elif abs(cx - x) < 50:
                        step = 0.1

                    if abs(cx - x) < 2:
                        if not np.isnan(dist):
                            if dist > 0.55:
                                msg.linear.x = 0.3
                                print("Move forward (speed: 0.3)")
                            elif dist > 0.46:
                                msg.linear.x = 0.1
                                print("Move forward (speed: 0.1)")
                            else:
                                msg.linear.x = 0.01
                                print("Move forward (speed: 0.01)")
                        elif self.forward_counter < 12:
                            msg.linear.x = 0.01
                            self.forward_counter += 1
                            print("Move forward manual", self.forward_counter)
                        elif self.forward_counter < 17:
                            msg.linear.x = 0.005
                            self.forward_counter += 1
                            print("Move forward manual", self.forward_counter)
                        else:
                            self.arm_control_pub.publish("extend")
                            print(f"Extending arm over the {color.upper()} cylinder")
                            self.speak(f"Im near the {color} cylinder.")
                            rospy.sleep(5)
                            self.arm_control_pub.publish("retract")
                            self.goal_reached = True
                            self.back_counter = 2
                    else:
                        if cx > x:
                            print(f"Turn right (speed: {step})")
                            msg.angular.z = -step
                        elif cy < x:
                            print(f"Turn left (speed: {step})")
                            msg.angular.z = step
            else:
                if self.back_counter > 0:
                    msg.linear.x = -0.2
                    self.back_counter -= 1
                    print("Move back", self.back_counter)
                else:
                    self.forward_counter = 0
                    self.move_forward = 0
                    self.last_turn = None
                    self.goal_reached = False
                    self.back_counter = None
                    self.state = "return_to_stored_pose"
                    print("Return to stored pose")

            self.twist_pub.publish(msg)

        self.fine_navigation_img_pub.publish(
            self.bridge.cv2_to_imgmsg(cv2.bitwise_and(image, image, mask=cv2.bitwise_not(mask)))
        )
        print()

    def move_to_ring(self, ring):
        msg = PoseStamped()
        msg.header.seq = self.rings.current.id
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose = ring.navigation_pose

        msg.pose.orientation = self.fix_angle(ring.obj_pose, msg.pose)

        self.pose_pub.publish(msg)
        self.waypoint_markers.markers.append(self.create_marker(self.seq + 100, msg.pose, b=1))

    def approach_ring(self):
        self.state = "moving_to_face"

        self.move_to_face(self.faces.current)
        self.speak(f"I just picked up the {self.faces.current.vaccine} vaccine.")
        return

        ring = self.rings.current

        cv_image = self.bridge.imgmsg_to_cv2(
            rospy.wait_for_message("/camera/rgb/image_raw", Image), "bgr8"
        )
        depth_image = rospy.wait_for_message("/camera/depth/image_raw", Image)
        depth_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")
        left = np.nanmean(
            depth_image[
                self.params["vertical_space"] : -self.params["vertical_space"],
                0 : self.params["horizontal_space"],
            ]
        )
        right = np.nanmean(
            depth_image[
                self.params["vertical_space"] : -self.params["vertical_space"],
                depth_image.shape[1] - self.params["horizontal_space"] :,
            ]
        )

        current_pose = self.get_current_pose()

        color = ring.color

        dist = math.sqrt(
            pow((current_pose.pose.position.x - ring.obj_pose.position.x), 2)
            + pow((current_pose.pose.position.y - ring.obj_pose.position.y), 2)
        )

        angle = self.to_euler(current_pose.pose.orientation)

        target_angle = self.fix_angle(ring.obj_pose, current_pose)
        target_angle = self.to_euler(target_angle)

        msg = Twist()

        print(
            f"Distance: {dist:.4}, Target distance: {self.ring_target_distance}, angle: {((angle/math.pi)*180):.4}, target angle: {((target_angle/math.pi)*180):.4}, left: {left:.4}, right: {right:.4}"
        )

        avoid_rotation_step = 0.5
        if not self.goal_reached:
            if self.bumper_hit:
                self.bumper_hit = False
                if dist > 0.3:
                    msg.linear.x = -0.4
                if dist > 0.15 and self.bumper_hit_counter < 2:
                    msg.linear.x = -0.4
                    self.bumper_hit_counter += 1
                else:
                    msg.linear.x = 0.04
                    self.goal_reached = True
                if self.bumper_hit_counter > 1:
                    self.ring_target_distance += 0.01
            elif ((np.isnan(left) or left < 0.45) and dist > 0.3) and (
                not ((np.isnan(left) and np.isnan(right))) or abs(left - right) > 0.2
            ):
                self.move_forward = 1
                msg.angular.z = -avoid_rotation_step
                self.last_turn = "right"
                print(f"Avoid, turn right (speed: {avoid_rotation_step})")
            elif ((np.isnan(right) or right < 0.45) and dist > 0.3) and (
                not ((np.isnan(left) and np.isnan(right))) or abs(left - right) > 0.2
            ):
                msg.angular.z = avoid_rotation_step
                self.last_turn = "left"
                self.move_forward = 1
                print(f"Avoid, turn left (speed: {avoid_rotation_step})")
            elif (self.move_forward > 0 and dist > 0.3) and (
                (not np.isnan(left) and not np.isnan(right)) or abs(left - right) > 0.2
            ):
                self.move_forward -= 1
                if dist > 0.35:
                    msg.linear.x = 0.3
                    print(f"Avoid, move forward (speed: 0.3)")
                elif dist > 0.3:
                    msg.linear.x = 0.22
                    print(f"Avoid, move forward (speed: 0.22)")
                elif dist > 0.25:
                    msg.linear.x = 0.2
                    print(f"Avoid, move forward (speed: 0.2)")
                elif dist > 0.2:
                    msg.linear.x = 0.1
                    print(f"Avoid, move forward (speed: 0.1)")
                else:
                    self.move_forward = 0
            elif (self.last_turn != None) and (
                (not np.isnan(left) and not np.isnan(right)) or abs(left - right) > 0.2
            ):
                self.last_turn = None
                if dist > 0.4:
                    if self.last_turn == "left":
                        msg.angular.z = -0.2
                        print("Avoid, rotate back right (speed: 0.2)")
                    else:
                        msg.angular.z = 0.2
                        print("Avoid, rotate back left (speed: 0.2)")
            else:
                if abs(target_angle - angle) > (math.pi / 180) * 10:
                    if angle > target_angle:
                        msg.angular.z = -0.4
                        print("Rotate right (speed: 0.4)")
                    elif angle < target_angle:
                        msg.angular.z = 0.4
                        print("Rotate left (speed: 0.4)")
                elif abs(target_angle - angle) > (math.pi / 180) * 5:
                    if angle > target_angle:
                        msg.angular.z = -0.2
                        print("Rotate right (speed: 0.2)")
                    elif angle < target_angle:
                        msg.angular.z = 0.2
                        print("Rotate left (speed: 0.2)")
                elif abs(target_angle - angle) > (math.pi / 180) * 2:
                    if angle > target_angle:
                        msg.angular.z = -0.05
                        print("Rotate right (speed: 0.05)")
                    elif angle < target_angle:
                        msg.angular.z = 0.05
                        print("Rotate left (speed: 0.05)")
                else:
                    if dist > 0.5:
                        msg.linear.x = 0.1
                        print("Move forward (speed: 0.1)")
                    elif dist > 0.4:
                        msg.linear.x = 0.05
                        print("Move forward (speed: 0.05)")
                    elif dist > 0.2:
                        msg.linear.x = 0.03
                        print("Move forward (speed: 0.03)")
                    elif dist > self.ring_target_distance:
                        msg.linear.x = 0.01
                        print("Move forward (speed: 0.01)")
                    else:
                        self.goal_reached = True

        else:
            if self.back_counter == None:
                print(f"Robot under the {color.upper()} ring")
                self.speak(f"Im under the {color} ring.")
                rospy.sleep(5)
                self.back_counter = 4
            elif self.back_counter > 0:
                self.back_counter -= 1
                msg.linear.x = -0.2
                print("Move back", self.back_counter)
            else:
                self.forward_counter = 0
                self.move_forward = 0
                self.last_turn = None
                self.goal_reached = False
                self.back_counter = None
                self.bumper_hit_counter = 0
                self.ring_target_distance = self.ring_default_distance
                self.arm_control_pub.publish("retract")
                self.state = "moving_to_face"
                self.move_to_face(self.faces.current)
                print("Return to stored pose")

        self.twist_pub.publish(msg)

        cv_image[
            self.params["vertical_space"] : -self.params["vertical_space"],
            0 : self.params["horizontal_space"],
        ] = (0, 0, 255)

        cv_image[
            self.params["vertical_space"] : -self.params["vertical_space"],
            cv_image.shape[1] - self.params["horizontal_space"] :,
        ] = (0, 0, 255)
        self.fine_navigation_img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))

        print()

    def move_to_face(self, face):
        msg = PoseStamped()
        msg.header.seq = face.id
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose = face.navigation_pose

        self.waypoint_markers.markers.append(self.create_marker(self.seq + 100, msg.pose, b=1))
        self.pose_pub.publish(msg)

    def get_face_info(self):
        res = self.dialog(0)
        self.faces.current.already_vaccinated = res.already_vaccinated or not res.wants_vaccine
        self.faces.current.doctor = res.doctor
        self.faces.current.hours_of_exercise = res.hours_of_exercise
        self.visited = True
        print("Dialog OK", res.already_vaccinated, res.wants_vaccine)
        # self.faces.current.already_vaccinated = False
        # self.faces.current.doctor = "red"
        # self.faces.current.hours_of_exercise = 10

        if not self.faces.current.mask:
            self.speak("Please wear a mask!")
        self.state = "read_digits"

    def read_digits(self):
        print("Read digits")
        res = self.extract_digits(0)
        if res.status == 0:
            print("Detected digits:", res.data)
            self.faces.current.age = res.data
            self.updated_pose = False
            self.state = "proccess_face"
        else:
            if self.updated_pose == False:
                self.updated_pose = True
                print("No digits found, fixing position")
                msg = Twist()
                msg.linear.x = -0.3
                self.twist_pub.publish(msg)
                rospy.sleep(2)
                self.move_to_face(self.faces.current)
                rospy.sleep(5)
                self.fix_heading(self.faces.current.navigation_pose)
                self.turn_counter = 0
            else:
                if self.turn_counter < 7:
                    self.turn_counter += 1
                    print("No digits found, fixing orientation")
                    msg = Twist()
                    msg.angular.z = 0.05
                    # msg.linear.x = 0.01
                    self.twist_pub.publish(msg)
                    rospy.sleep(1)
                else:
                    self.updated_pose = False

    def fix_heading(self, target):
        while True:
            target_angle = self.to_euler(target.orientation)
            current_pose = self.get_current_pose()
            angle = self.to_euler(current_pose.pose.orientation)
            angle_diff = abs(angle - target_angle) % (2 * math.pi)
            msg = Twist()
            print("Diff: ", angle_diff, "Angle: ", angle, "Target_angle: ", target_angle)
            if angle_diff > (math.pi / 180) * 5:
                if target_angle > angle:
                    if abs(target_angle - angle) > (math.pi * 2 - abs(angle - target_angle)):
                        msg.angular.z = -0.1
                    else:
                        msg.angular.z = 0.1
                else:
                    if abs(target_angle - angle) > (math.pi * 2 - abs(angle - target_angle)):
                        msg.angular.z = 0.1
                    else:
                        msg.angular.z = -0.1
                self.twist_pub.publish(msg)
                rospy.sleep(1)
            else:
                return

    def fix_heading_cylinder(self):
        color = self.cylinders.current.color

        while True:
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
                hsv,
                *limits.get(color, (np.array([0, 0, 0]), np.array([110, 255, 255]))),
            )
            countour, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(countour) > 0:
                depth_image = rospy.wait_for_message("/camera/depth/image_raw", Image)
                depth_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")

                cont = max(countour, key=cv2.contourArea)
                y = int(image.shape[0] / 2)
                x = int(image.shape[1] / 2)
                m = cv2.moments(cont)
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])

                cv2.circle(image, (cx, cy), 3, (0, 255, 0))
                cv2.drawContours(image, [cont], -1, (0, 255, 0), 2)

                msg = Twist()

                step = 0.15
                if abs(cx - x) < 5:
                    step = 0.005
                elif abs(cx - x) < 20:
                    step = 0.01
                elif abs(cx - x) < 30:
                    step = 0.03
                elif abs(cx - x) < 50:
                    step = 0.1

                if abs(cx - x) > 2:
                    if cx > x:
                        print(f"Turn right (speed: {step})")
                        msg.angular.z = -step
                    elif cy < x:
                        print(f"Turn left (speed: {step})")
                        msg.angular.z = step
                    self.twist_pub.publish(msg)
                    self.fine_navigation_img_pub.publish(
                        self.bridge.cv2_to_imgmsg(
                            cv2.bitwise_and(image, image, mask=cv2.bitwise_not(mask))
                        )
                    )
                else:
                    return

    def check_social_distance(self):
        faces_copy = copy.deepcopy(self.faces.data)
        for e, ke in faces_copy.items():
            e_angle = self.to_euler(e.navigation_pose.orientation)
            for f, kf in faces_copy.items():
                if f == e:
                    continue
                f_angle = self.to_euler(f.navigation_pose.orientation)
                if (e.warned and f.warned) or abs(f_angle - e_angle) > math.pi / 4:
                    continue
                dist = np.sqrt(
                    np.power(e.obj_pose.position.x - f.obj_pose.position.x, 2)
                    + np.power(e.obj_pose.position.y - f.obj_pose.position.y, 2)
                )

                if dist < 1:
                    msg = PoseStamped()
                    msg.pose.position.x = (
                        e.navigation_pose.position.x + f.navigation_pose.position.x
                    ) / 2
                    msg.pose.position.y = (
                        e.navigation_pose.position.y + f.navigation_pose.position.y
                    ) / 2
                    msg.pose.orientation = Quaternion(
                        *list(
                            tf.transformations.quaternion_from_euler(0, 0, (f_angle + e_angle) / 2)
                        )
                    )

                    self.faces.data[ke].warned = True
                    self.faces.data[kf].warned = True
                    e.warned = True
                    f.warned = True

                    print("MOVE TO WARNING")

                    self.interupt_plan()
                    self.state = "moving_to_warning_safety_distance"
                    msg.header.seq = 200 + e.id
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "map"

                    self.waypoint_markers.markers.append(
                        self.create_marker(self.seq + 200, msg.pose, r=1)
                    )
                    self.pose_pub.publish(msg)

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

    def to_euler(self, quat):
        angle = tf.transformations.euler_from_quaternion(
            [
                quat.x,
                quat.y,
                quat.z,
                quat.w,
            ]
        )[2]

        if angle < 0:
            angle += 2 * math.pi

        return angle

    def result_sub_callback(self, data):
        res_state = data.status.status
        if self.state == "moving_to_waypoint":
            if res_state in (3, 4):
                self.seq += 1
                self.waypoints.current.visited = True
                self.state = "get_next_waypoint"
                self.fade_markers()

        elif self.state == "moving_to_cylinder":
            if res_state == 3:
                self.fix_heading_cylinder()
                self.state = "read_qr_code"
            elif res_state == 4:
                self.cylinders.reset_current()
                self.state = "return_to_stored_pose"

        elif self.state == "moving_to_ring":
            if res_state == 3:
                self.fix_heading(self.rings.current.navigation_pose)
                self.arm_control_pub.publish("grab_vaccine")
                self.state = "approach_ring"
            elif res_state == 4:
                self.rings.reset_current()
                self.state = "return_to_stored_pose"

        elif self.state == "moving_to_face":
            if res_state == 3:
                self.fix_heading(self.faces.current.navigation_pose)
                if self.delivering_vaccine:
                    self.state = "give_vaccine"
                else:
                    self.state = "initial_dialog"
            elif res_state == 4:
                self.faces.reset_current()
                self.state = "return_to_stored_pose"

        if self.state == "moving_to_warning_safety_distance":
            if res_state in (3, 4):
                self.state = "warning_safety_distance"

        elif self.state == "return_home":
            if res_state == 3:
                self.state = "end"
                print(self.state)

    def cylinder_sub_callback(self, data):
        for e in data.data:
            self.cylinders.add(Cylinder(e.id, e.obj_pose, e.navigation_pose, e.color))

    def ring_sub_callback(self, data):
        for e in data.data:
            self.rings.add(Ring(e.id, e.obj_pose, e.navigation_pose, e.color))
        if self.searching_ring:
            ring = self.rings.get_by(self.faces.current.vaccine, "vaccine")
            if ring != None:
                self.rings.current = ring
                self.ring_found = True

    def face_sub_callback(self, data):
        for e in data.data:
            self.faces.add(Face(e.id, e.obj_pose, e.navigation_pose, e.mask))

    def bumper_callback(self, data):
        if data.state == 1:
            self.bumper_hit = True


class ObjectContainer:
    def __init__(self):
        self.data = dict()
        self.new_entry = list()
        self.last_id = -1
        self.current = None

    def is_new_data(self):
        return len(self.new_entry) > 0

    def get_last(self):
        if len(self.new_entry) > 0:
            self.current = self.data[self.new_entry.pop()]
            self.last_id = self.current.id
            return self.current
        return None

    def add(self, obj):
        if obj.id in self.data.keys():
            self.update(obj)
        else:
            self.data[obj.id] = obj
            self.new_entry.append(obj.id)

    def update(self, obj):
        self.data[obj.id].update(obj)

    def reset_current(self):
        self.last_id = -1
        self.new_entry.insert(0, self.current.id)

    def get_by(self, value, by):
        if by == "color":
            for e in self.data.values():
                if e.color == value:
                    return e
            return None
        elif by == "vaccine":
            for e in self.data.values():
                if e.color in value.lower():
                    return e
            return None
        return None


class Cylinder:
    def __init__(self, id, obj_pose, navigation_pose, color):
        self.id = id
        self.obj_pose = obj_pose
        self.navigation_pose = navigation_pose
        self.color = color
        self.vaccine_predictor = None
        self.clf = None

    def update(self, obj):
        self.obj_pose = obj.obj_pose
        self.navigation_pose = obj.navigation_pose
        self.color = obj.color

    def add_clf(self, url):
        self.clf = VaccinePredictor(url)

    def predict(self, age, hours_exercise):
        return self.clf.predict(age, hours_exercise)


class Ring:
    def __init__(self, id, obj_pose, navigation_pose, color):
        self.id = id
        self.obj_pose = obj_pose
        self.navigation_pose = navigation_pose
        self.color = color

    def update(self, obj):
        self.obj_pose = obj.obj_pose
        self.navigation_pose = obj.navigation_pose
        self.color = obj.color


class Face:
    def __init__(self, id, obj_pose, navigation_pose, mask):
        self.id = id
        self.obj_pose = obj_pose
        self.navigation_pose = navigation_pose
        self.mask = mask
        self.age = None
        self.hours_exercise = None
        self.warned = False
        self.doctor = None
        self.vaccine = None
        self.already_vaccinated = False
        self.proccessed = False

    def update(self, obj):
        self.obj_pose = obj.obj_pose
        self.navigation_pose = obj.navigation_pose


class VaccinePredictor:
    def __init__(self, url):
        self.url = url
        self.prep_data()
        self.train()

    def prep_data(self):
        self.data = pd.read_csv(
            io.StringIO(requests.get(self.url).content.decode("utf-8")),
            names=["age", "hours_exercise", "right_vaccine"],
        )
        self.le = LabelEncoder()
        self.data["right_vaccine_enc"] = self.le.fit_transform(self.data["right_vaccine"])
        self.X = self.data.drop(columns=["right_vaccine", "right_vaccine_enc"])
        self.y = self.data.drop(columns=["age", "hours_exercise", "right_vaccine"])

    def train(self):
        X_train, X_test, y_train, y_test = train_test_split(
            self.X, self.y, test_size=0.3, random_state=42
        )
        self.clf = DecisionTreeClassifier()
        self.clf.fit(X_train, y_train)
        y_pred_test = self.clf.predict(X_test)
        print(f"Accuracy on test dataset: {accuracy_score(y_test, y_pred_test)}")

    def predict(self, age, hours_exercise):
        res = self.clf.predict([[56.56565656565657, 21.224489795918366]])
        return self.le.inverse_transform(res)[0]


class Waypoint:
    def __init__(self, pose):
        self.pose = pose
        self.visited = False


class Waypoints:
    def __init__(self):
        self.data = list()
        self.current = None
        self.visited = 0
        self.history = list()

    def get_available(self):
        return [e for e in self.data if not e.visited]

    def reset(self):
        for e in self.data:
            e.visited = False
        self.visited = 0
        self.history = list()

    def add(self, pose):
        self.data.append(pose)

    def set_current(self, waypoint):
        self.current = waypoint
        self.history.append(waypoint)


if __name__ == "__main__":
    ms = Mover()
