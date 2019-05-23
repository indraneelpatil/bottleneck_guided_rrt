#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from datetime import datetime

from matplotlib.animation import FuncAnimation
from random import randrange
import matplotlib.pyplot as plt
import numpy as np
import time


from std_msgs.msg import Float64

Error = 0;
previous_error = 0;    #previous error
change_y = 0;
integral = 0;
derivative = 0;

desiredAngle = 0.00;

x_data, y_data = [], []

#figure = plt.figure()
#line, = plt.plot_date(x_data, y_data, '-')


Kp = 0.02;                               
Ki = 0.001; 
Kd = 0.0015; 
program_start=time.time()

def callback(data):
	global integral,Error, previous_error,change_y,derivative,desiredAngle,Kp,Ki,Kd
	global robot, move_group,scene,display_trajectory_publisher,program_start,wpose
	#rospy.loginfo("I heard %s", data.data)
	actualAngle=data.data
	Error=desiredAngle- actualAngle
	integral=integral+Error
	derivative=Error-previous_error
	change_y = (Kp * Error) + (Ki * integral) + (Kd * derivative) 
	# Set Max Range for change_y and yout

	#wpose = move_group.get_current_pose().pose
		
	current_y=0
	
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.x = wpose.orientation.x
	pose_goal.orientation.y = wpose.orientation.y
	pose_goal.orientation.z = wpose.orientation.z
	pose_goal.orientation.w = wpose.orientation.w
	pose_goal.position.x = wpose.position.x
	pose_goal.position.y = wpose.position.y+change_y
	pose_goal.position.z = wpose.position.z
	move_group.set_pose_target(pose_goal)
	#move_group.set_named_target("test1")
	
	plan = move_group.plan()
	move_group.go(wait=True)
	# Calling `stop()` ensures that there is no residual movement
	#move_group.stop()
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	#move_group.clear_pose_targets()
	#move_group.execute(plan, wait=True)

	rospy.loginfo(change_y)
	#wpose2=move_group.get_current_pose().pose
	#print(wpose2)

	previous_angle=actualAngle
	previous_error=Error
	#figure = pyplot.figure()

	#time_instant=time.time()
	#time_var=int(time_instant-program_start)
	#plt.axis([0, time_var,-30, 30])
	#x_data.append(time_var)
	#y_data.append(data.data)
	#plt.plot(x_data, y_data)
	#plt.pause(0.0000001) #Note this correction
	#figure.gca().relim()
	#figure.gca().autoscale_view()
	#print data.data
	
	#plt.show()
	



  
def listener():
	global robot, move_group,scene,display_trajectory_publisher,wpose

	rospy.init_node('controller', anonymous=True)
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group_name = "manipulator"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	# Display trajectories in Rviz
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
	#joint_goal = move_group.get_current_joint_values()
	wpose = move_group.get_current_pose().pose
	print(wpose)
	rospy.Subscriber("/angle_topic", Float64, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
    listener()