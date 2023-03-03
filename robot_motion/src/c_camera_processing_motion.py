#!/usr/bin/env python3

'''
Currently in development
'''
from __future__ import print_function

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class NiryoArm_move_group_interface(object):

    def __init__(self):
        super(NiryoArm_move_group_interface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_niryo_arm", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/gazebo_camera/image_raw", Image, self.callback)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "tool"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # print("\nEnd Effector link Name : %s" %   move_group.get_end_effector_link())
        # print("Available Planning Groups:",       robot.get_all_robot_groups())
        print("Current Joint State",              robot.get_current_state())

        self.robot                        = robot
        self.scene                        = scene
        self.move_group                   = move_group
        self.planning_frame               = move_group.get_planning_frame()
        # self.eef_link                     = move_group.get_end_effector_link()
        self.group_name                   = group_name
        # self.all_robot_groups                  = robot.get_all_robot_groups()

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Gray Image", gray_image)
        cv2.waitKey(3)
    def send_angles_to_joints(self):
        """
        Sending Individual Joints angles
        """
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        if (self.group_name=='arm' ):
            ## Arm
            joint_goal[0] = 0
            joint_goal[1] = 0.2
            joint_goal[2] = 0
            joint_goal[3] = 0.004
            joint_goal[4] = 0
            joint_goal[5] = 0.006
        else:
            ## Gripper
            ###Limits for grippers are from -0.01 to 0.006
            joint_goal[0] = 0.006
            joint_goal[1] = 0.006

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        ## testing
        current_joints = move_group.get_current_joint_values()
        # return all_close(joint_goal, current_joints, 0.01)


    def pick_a_cube(self):
        print("Opening Tool")
        # Opening Tool
        group_name = "tool"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0.005
        joint_goal[1] = 0.005

        move_group.go(joint_goal, wait=True)
        move_group.stop()


        print("Moving Robot")
        # Moving Robot to a Cube
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.25
        pose_goal.position.y = 0.035
        pose_goal.position.z = 0.037
        move_group.set_pose_target(pose_goal)
        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()


        # print("Closing Tool")
        # # Close Tool
        # move_group = "tool"
        # joint_goal = move_group.get_current_joint_values()
        # joint_goal[0] = 0.003
        # joint_goal[1] = 0.003

        # move_group.go(joint_goal, wait=True)
        # move_group.stop()

        # print("Moving Robot Back")
        # # Moving Robot to another position
        # move_group = "arm"
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 1.0
        # pose_goal.position.x = 0.3
        # pose_goal.position.y = 0.1
        # pose_goal.position.z = 0.1
        # move_group.set_pose_target(pose_goal)
        # move_group.go(wait=True)
        # move_group.stop()
        # move_group.clear_pose_targets()
def main():
    niryo_arm = NiryoArm_move_group_interface()
    print(" Starting Camera and moving Robot End Effect " )
    niryo_arm.pick_a_cube()


if __name__ == "__main__":
    main()

