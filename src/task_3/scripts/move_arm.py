#!/usr/bin/python3

import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String


class Arm_Mover:
    def __init__(self):

        rospy.init_node("arm_mover", anonymous=True)

        self.arm_movement_pub = rospy.Publisher(
            "/turtlebot_arm/arm_controller/command", JointTrajectory, queue_size=1
        )
        self.arm_user_command_sub = rospy.Subscriber("/arm_command", String, self.new_user_command)

        # Just for controlling wheter to set the new arm position
        self.user_command = None
        self.send_command = False

        self.positions = {
            "extend": [0.3, 1.55, 0.1, 0],
            "grab_vaccine": [0.4, 2.2, -2.75, 1.5],
            "retract": [0, -2.1, 2.55, 1.1],
        }

        self.joints = [
            "arm_shoulder_pan_joint",
            "arm_shoulder_lift_joint",
            "arm_elbow_flex_joint",
            "arm_wrist_flex_joint",
        ]

    def new_user_command(self, data):
        self.user_command = data.data.strip()
        print(self.user_command in self.positions.keys())
        if self.user_command in self.positions.keys():
            self.send_command = True
        else:
            print("Invalid pose name")

    def update_position(self):
        # Only if we had a new command
        if self.send_command:
            trajectory = self.construct_trajectory(self.user_command)
            print(trajectory)
            self.arm_movement_pub.publish(trajectory)
            self.send_command = False

    def construct_trajectory(self, pose_name):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        trajectory.points = [
            JointTrajectoryPoint(
                positions=self.positions[pose_name], time_from_start=rospy.Duration(1)
            )
        ]
        return trajectory


if __name__ == "__main__":
    am = Arm_Mover()
    time.sleep(0.5)
    am.arm_movement_pub.publish(am.construct_trajectory("retract"))
    print("Retracted arm!")

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        am.update_position()
        r.sleep()
