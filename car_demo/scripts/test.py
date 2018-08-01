#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations
import gazebo_msgs.srv
import math
import numpy

target_cars = [
'prius', 
'car1_0_clone',
'car1_0',
'car4',
'car7_0',
'car6_1',
'car3_0',
'car5_0',
]

def pose_to_matrix(pose):
    translation = tf.transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
    rotation = tf.transformations.quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    return translation.dot(rotation)

def xyz_to_mat(x, y, z):
    return tf.transformations.translation_matrix((x, y, z))

def rpy_to_mat(r, p, y):
    return tf.transformations.euler_matrix(r, p, y)

def matrix_to_pose(matrix):
    position = geometry_msgs.msg.Point(*tuple(tf.transformations.translation_from_matrix(matrix)))
    orientation = geometry_msgs.msg.Quaternion(*tuple(tf.transformations.quaternion_from_matrix(matrix)))
    return geometry_msgs.msg.Pose(position, orientation)

rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', gazebo_msgs.srv.GetLinkState);

def get_link_pose(link_name):
    return get_link_state(link_name, '').link_state.pose

# 3 wont work.. .
# the number..
# 7 5 4 1
# 0 6 2 3     3 wont work because it is too low..
while(1):
	num = int(sys.stdin.readline())
	if num>=0 and num <8:
		charger_name = '%s::charge_oncar::link_2' % target_cars[num]
		print charger_name
		charger_pose = get_link_pose(charger_name)
		charger_mat = pose_to_matrix(charger_pose)

		robot_trans = xyz_to_mat(19.5, -315.0, 1.76)
		robot_rot = rpy_to_mat(3.1416, 0, 0)
		robot_mat = robot_trans.dot(robot_rot)
		# Should be the same, or close
		#robot_pose = get_link_pose('robot::lateral_slider_link')
		#robot_mat = pose_to_matrix(robot_pose)

		rel_mat = numpy.linalg.inv(robot_mat).dot(charger_mat)

		offset = tf.transformations.translation_matrix((0.01, 0, 0.6))  #with large offset
		turnaround = tf.transformations.euler_matrix(0, math.pi, 0)

		target_matrix = rel_mat.dot(offset).dot(turnaround)
		target_pose = matrix_to_pose(target_matrix)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group = moveit_commander.MoveGroupCommander("manipulator")
		display_trajectory_publisher = rospy.Publisher(
				                    '/ur5/move_group/display_planned_path',
				                    moveit_msgs.msg.DisplayTrajectory,
				                    queue_size=20)

		print "============ Reference frame:", group.get_planning_frame()
		print "============ End effector:", group.get_end_effector_link()
		print "============ Robot Groups:", robot.get_group_names()

		group.clear_pose_targets()
		group.set_planning_time(5.0)
		print "default plan time %f" % group.get_planning_time()
		group.set_goal_position_tolerance(0.005)

		#firstly move to center
		group.set_goal_position_tolerance(0.005)
		group.set_goal_joint_tolerance(0.005)
		group.set_planner_id("PRMkConfigDefault")
		joints_value = group.get_current_joint_values()
		#joints_value[0] = target_pose.position.y  current value.
		joints_value[1] = 0

		group.set_joint_value_target(joints_value)
		group.go(wait=True)
		rospy.sleep(5)
		
		#then control x,y  x to zero, y to the y position with offset
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group = moveit_commander.MoveGroupCommander("manipulator")
                #joints_value = group.get_joint_value_target()
		joints_value = group.get_current_joint_values()
		joints_value[0] = target_pose.position.y
		joints_value[1] = 0
		joints_value[2] = 0
		joints_value[3] = -1.5707
		joints_value[4] = 0
		joints_value[5] = -1.5707
		joints_value[6] = 0
		joints_value[7] = 0

		group.set_joint_value_target(joints_value)
		group.go(wait=True)
		rospy.sleep(5)
		#then approach
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group = moveit_commander.MoveGroupCommander("manipulator")
		offset = tf.transformations.translation_matrix((0, 0, 0.06))  #with less offset

		target_matrix = rel_mat.dot(offset).dot(turnaround)
		target_pose = matrix_to_pose(target_matrix)
		group.set_goal_position_tolerance(0.005)
		group.set_planning_time(10.0)
		group.set_goal_position_tolerance(0.0005)
		group.set_planner_id("PRMkConfigDefault")
		group.clear_pose_targets()
		group.set_pose_target(target_pose)

		#plan1 = group.plan()

		group.go(wait=True)
		print "============================Waiting for next position"
	else:
		moveit_commander.roscpp_shutdown()
