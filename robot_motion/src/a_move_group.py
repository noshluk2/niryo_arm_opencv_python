#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi

class MoveGroupPythonInterfaceniryo_arm(object):

    def __init__(self):
        super(MoveGroupPythonInterfaceniryo_arm, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_niryo_arm", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(self.group_name)

        print("\nEnd Effector link Name : %s" %   move_group.get_end_effector_link())
        print("Available Planning Groups:",       robot.get_group_names())
        print("Current Joint State",              robot.get_current_state())

        self.robot                        = robot
        self.scene                        = scene
        self.move_group                   = move_group
        self.planning_frame               = move_group.get_planning_frame()
        self.eef_link                     = move_group.get_end_effector_link()
        self.group_names                  = robot.get_group_names()

    def send_angles_to_joints(self):
        """
        Sending Individual Joints angles
        """
        tau = 2.0 * pi
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        if (self.group_name=="arm" ):
            ## Arm
            joint_goal[0] = 0
            joint_goal[1] = 0.2
            joint_goal[2] = 0
            joint_goal[3] = -tau / 4
            joint_goal[4] = 0
            joint_goal[5] = tau / 6
        else:
            ## Gripper
            ###Limits for grippers are from -0.01 to 0.006
            joint_goal[0] = 0.006
            joint_goal[1] = 0.006

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        ## testing
        current_joints = move_group.get_current_joint_values()


    def send_goal_to_end_effect(self):
        """
        Sending Goal location for End effector
        """
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)


        move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

def main():
    niryo_arm = MoveGroupPythonInterfaceniryo_arm()
    print(" Starting Robot_motion " )
    niryo_arm.send_angles_to_joints()
    # niryo_arm.send_goal_to_end_effect()


if __name__ == "__main__":
    main()

