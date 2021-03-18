#! /usr/bin/env python
import sys
import rospy
import moveit_commander

import geometry_msgs.msg
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group = moveit_commander.MoveGroupCommander("hand")

# Put the arm in the start position
arm_group.set_named_target("start")
plan1 = arm_group.go()

# Open the gripper
hand_group.set_named_target("open")
plan2 = hand_group.go()

# put the arm at the 1st grasping position
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.15
pose_target.position.y = 0.0
pose_target.position.z = 1.25
arm_group.set_pose_target(pose_target)
#plan1 = arm_group.plan()
plan1 = arm_group.go()

# put the arm at the 2nd grasping position
pose_target.position.z = 1.125
arm_group.set_pose_target(pose_target)
#plan1 = arm_group.plan()
plan1 = arm_group.go()

# close the gripper
hand_group.set_named_target("close")
plan2 = hand_group.go()

# put the arm at the 3rd grasping position
pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
#plan1 = arm_group.plan()
plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()