#!/usr/bin/env python3

import rospy
import math
# import the messages for reading the joint positions and sending joint commands
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from std_msgs.msg import Bool
from surgery_sim.msg import PedalEvent

init_flag = False
right = False
left = False
middle = False

def pedal_callback(data):
	global left
	global middle
	global right
	if data.left_pedal == 1:
		left = True
		middle = False
		right = False
	if data.middle_pedal == 1:
		middle = True
		left = False
		right = False
	if data.right_pedal == 1:
		right = True
		left = False
		middle = False
	
def init_callback(data):
	global init_flag
	init_flag = data.data

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('manual_initialization', anonymous = True)
	# define a subscriber to read pedal press
	pedal_sub = rospy.Subscriber('/pedal', PedalEvent, pedal_callback)
	# define a subscriber to read init flag
	init_sub = rospy.Subscriber('/init_flag', Bool, init_callback)
	# add a publisher for sending joint position commands
	pos_pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	begin = False;

	# define a joint trajectory variable for sending the control commands
	pos_cmd = JointTrajectory()
	pos_cmd_point = JointTrajectoryPoint()
	# just a quick solution to complete the message template
	pos_cmd.joint_names.append('elbow_joint')
	pos_cmd.joint_names.append('shoulder_lift_joint') # shoulder
	pos_cmd.joint_names.append('shoulder_pan_joint') # base
	pos_cmd.joint_names.append('wrist_1_joint')
	pos_cmd.joint_names.append('wrist_2_joint')
	pos_cmd.joint_names.append('wrist_3_joint')
	
	# initialize the position command to zero
	for joint_no in range(6):
		pos_cmd_point.positions.append(0.0)
	# set the ideal time to destination
	pos_cmd_point.time_from_start = rospy.Duration(1.0) # here one second 

	pos_cmd_point.positions[0] = 0.824
	pos_cmd_point.positions[1] = -0.5436
	pos_cmd_point.positions[2] = 1.376
	pos_cmd_point.positions[3] = -1.85
	pos_cmd_point.positions[4] = -1.5644
	pos_cmd_point.positions[5] = -1.752
	# add the trajectory point to the command
	pos_cmd.points.append(pos_cmd_point)
	# define a message header	
	header = Header()

	
	while not rospy.is_shutdown():
		if left:
			begin = True
		else:
			begin = False
		if init_flag and begin:
			# update the header with the most recent time stamp
			header.stamp = rospy.Time.now()
			# use the most recent header in the position command message
			pos_cmd.header = header
			# publish the message
			pos_pub.publish(pos_cmd)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
