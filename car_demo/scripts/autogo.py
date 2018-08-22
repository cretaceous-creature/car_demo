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
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
import math

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

transtmp = TransformStamped()

parking_midline_x = 19.5

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

def transcallback(data):
	global transtmp 
	transtmp = data
	print(transtmp.transform.translation.x,transtmp.transform.translation.y,transtmp.transform.translation.z)

rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
rospy.Subscriber('/matching_trans', TransformStamped, transcallback)

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

		#offset = tf.transformations.translation_matrix((0.01, 0, 0.6))  #with large offset
		offset = tf.transformations.translation_matrix((0.0, 0, 0.0))  #with large offset
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
		group.set_planning_time(2.0)
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
		rospy.sleep(1)
		
		#then control x,y  x to zero, y to the y position with offset
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group = moveit_commander.MoveGroupCommander("manipulator")
#		joints_value = group.get_joint_value_target()
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
		rospy.sleep(1)
		# trying to search
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group = moveit_commander.MoveGroupCommander("manipulator")
		offset = tf.transformations.translation_matrix((0, 0.2, 0.4))
		#offset = tf.transformations.translation_matrix((0, 0.0, 0.0))
                #with less offset

		target_matrix = rel_mat.dot(offset).dot(turnaround)
		target_pose = matrix_to_pose(target_matrix)
                
                
                
		group.set_goal_position_tolerance(0.005)
		group.set_planning_time(5.0)
		group.set_goal_position_tolerance(0.0005)
		group.set_planner_id("PRMkConfigDefault")
		group.clear_pose_targets()
#                print(target_pose.position)
                search_x = target_pose.position.x
                print(target_pose.orientation)
                target_pose.position.x = 0
                if target_pose.orientation.z < 0.5:
                    target_pose.position.y = target_pose.position.y - 0.6
                else:
                    target_pose.position.y = target_pose.position.y + 0.6
                    
		group.set_pose_target(target_pose)

		#plan1 = group.plan()
                #target_pose.position.y is the parking position,
                #target_pose.position.x is the lateral position, 0 means in the mid
                #the search x to be send to 5*sign(target_pose.position.x) with a limit

		group.go(wait=True)
                rospy.sleep(1)
		
                #now control the arm to search...
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group = moveit_commander.MoveGroupCommander("manipulator")
		joints_value = group.get_current_joint_values()
                while (1):
                    moveoder_all = sys.stdin.readline()
                    moveoder = moveoder_all[0]
                    if moveoder=='q':
                        break;
                    elif moveoder=='w':
                        joints_value[0] +=  0.1
                        group.set_joint_value_target(joints_value)
                    elif moveoder=='s':
                        joints_value[0] -=  0.1
                        group.set_joint_value_target(joints_value)
                    elif moveoder=='a':
                        joints_value[1] +=  0.1
                        group.set_joint_value_target(joints_value)
                    elif moveoder=='d':
                        joints_value[1] -=  0.1
                        group.set_joint_value_target(joints_value)
                    elif moveoder=='x':
                        joints_value[2] +=  0.1
                        group.set_joint_value_target(joints_value)
                    elif moveoder=='c':
                        joints_value[2] -=  0.1
                        group.set_joint_value_target(joints_value)
                    elif moveoder=='g':
						#here we automatically attach the robot to the charger
						current_pose = group.get_current_pose()
						offset = tf.transformations.translation_matrix((0, 0.05, 0.05))
						target_matrix = rel_mat.dot(offset).dot(turnaround)
						target_pose = matrix_to_pose(target_matrix)
						print(current_pose.pose.position.x,current_pose.pose.position.y,
						current_pose.pose.position.z)
						#print the position
						print(target_pose.position.x,target_pose.position.y,
						target_pose.position.z)		
						#print the orientation
						print(current_pose.pose.orientation.x,current_pose.pose.orientation.y,
						current_pose.pose.orientation.z,current_pose.pose.orientation.w)
						(roll,pitch,yaw) = tft.euler_from_quaternion([current_pose.pose.orientation.x,current_pose.pose.orientation.y,
						current_pose.pose.orientation.z,current_pose.pose.orientation.w])
						print("current roll,pitch,yaw")
						print(roll,pitch,yaw)
						#get currentpose matrix...
						currentmatrix = pose_to_matrix(current_pose.pose)
						currentrot = tft.quaternion_matrix([current_pose.pose.orientation.x, current_pose.pose.orientation.y,
						current_pose.pose.orientation.z, current_pose.pose.orientation.w])
						#rotation
						rotmat = tft.quaternion_matrix([transtmp.transform.rotation.x, transtmp.transform.rotation.y,
						transtmp.transform.rotation.z, transtmp.transform.rotation.w])
						#transform
						transmat = xyz_to_mat(transtmp.transform.translation.x, transtmp.transform.translation.y, 
						transtmp.transform.translation.z)
						#transmatrix
						transmatrix = transmat.dot(rotmat)
						#then currentpose move by this transmatrix..
						newmat = numpy.linalg.inv(currentmatrix).dot(transmatrix)
						#turn mat to pose		
						#offset = tf.transformations.translation_matrix((0.01, 0, 0.6))  #with large offset
						#turnaround = tf.transformations.euler_matrix(0, math.pi, 0)
						#target_matrix = rel_mat.dot(offset).dot(turnaround)
						new_pose = matrix_to_pose(newmat)
						print(new_pose.position.x,new_pose.position.y,new_pose.position.z)
						print(new_pose.orientation.x,new_pose.orientation.y,new_pose.orientation.z,new_pose.orientation.w)



                        #new method
						eelink_x = - transtmp.transform.translation.x
						eelink_y = - transtmp.transform.translation.y
						eelink_z =  transtmp.transform.translation.z
						eelink_mat = xyz_to_mat(-transtmp.transform.translation.x, -transtmp.transform.translation.y + 0.06, 
						transtmp.transform.translation.z)
						eelink_trans = eelink_mat.dot(currentrot)
						eelink_pose = matrix_to_pose( numpy.linalg.inv(currentrot).dot(eelink_trans) )
						print(eelink_pose.position.x,eelink_pose.position.y,eelink_pose.position.z)


						#target_pose.position.y += -math.cos(roll) * eelink_z 
						#target_pose.position.z += math.sin(roll) * eelink_z 
						#target_pose.position.x += math.cos() * eelink_x
						
						group.set_goal_position_tolerance(0.005)
						group.set_planning_time(5.0)
						group.set_goal_position_tolerance(0.0005)
						group.set_planner_id("PRMkConfigDefault")
						group.clear_pose_targets()
						group.set_pose_target(target_pose)



		    group.go(wait=True)
		    rospy.sleep(0.5)

                        
		print "============================Waiting for next position"
	else:
		moveit_commander.roscpp_shutdown()
