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
group = moveit_commander.MoveGroupCommander("xyeffector")
display_trajectory_publisher = rospy.Publisher(
                                    '/ur5/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

print "============ Reference frame: %s" % group.get_planning_frame()
print "============ End effector: %s" % group.get_end_effector_link()
print "============ Robot Groups:"
print robot.get_group_names()
print robot.get_current_state()
print "default plan time %f" % group.get_planning_time()

group.set_goal_position_tolerance(0.0005)
group.set_goal_joint_tolerance(0.0005)

joints_value = group.get_joint_value_target()

joints_value[0] = 3.3
joints_value[1] = 0.9

group.set_joint_value_target(joints_value)

#group.set_planner_id("PRMkConfigDefault")

#plan1 = group.plan()

group.go(wait=True)

moveit_commander.roscpp_shutdown()
