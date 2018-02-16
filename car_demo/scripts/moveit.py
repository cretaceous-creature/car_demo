#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher(
                                    '/ur5/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

print "============ Reference frame: %s" % group.get_planning_frame()
print "============ End effector: %s" % group.get_end_effector_link()
print "============ Robot Groups:"
print robot.get_group_names()
print robot.get_current_state()

group.clear_pose_targets()

pose_target = geometry_msgs.msg.Pose()
quat = [ -0.3043807, -0.5272029, 0.3966767, 0.6870641 ]
pose_target.orientation.x = quat[0]
pose_target.orientation.y = quat[1]
pose_target.orientation.z = quat[2]
pose_target.orientation.w = quat[3]
pose_target.position.x = 1.18
pose_target.position.y = 3.58
pose_target.position.z = 0.6
group.set_pose_target(pose_target)

group.set_planner_id("PRMkConfigDefault")

#plan1 = group.plan()

group.go(wait=True)

moveit_commander.roscpp_shutdown()
