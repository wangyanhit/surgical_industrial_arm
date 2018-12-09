#! /usr/bin/env python


import rospy
import actionlib
import random
import modern_robotics as mr
import numpy as np
from std_msgs.msg import Int16, Int16MultiArray
from math import pi
import copy
from control_msgs.msg import JointTrajectoryActionGoal, JointTrajectoryAction, FollowJointTrajectoryActionGoal,\
    FollowJointTrajectoryAction, FollowJointTrajectoryFeedback
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from servo_cmd.srv import ServoTrajectory, ServoTrajectoryRequest, ServoTrajectoryResponse
from servo_cmd.msg import test_msg

class ServoTrajectoryController:
    def __init__(self, n):
        self.node = n
        self.srv_servo_trajectory = rospy.Service('servo_trajectory', ServoTrajectory, self.servo_trajectory_cb)
        self.servo_joint_state_pub = rospy.Publisher('servo_joint_states', JointState, queue_size=10)
	self.servo_command_pub = rospy.Publisher('ServoPoseSerial', Int16MultiArray, queue_size=10)
	self.servo_joint_state_sub = rospy.Subscriber('m0', test_msg, self.servo_m0_Cb)
	self.servo = [0.0,0.0,0.0,0.0]
	self.traj = ServoTrajectoryRequest().points
	self.count = 0

    def servo_trajectory_cb(self, req):
        print("Received servo action message: {}".format(req))

	self.traj = req.points
	
 	res = ServoTrajectoryResponse()
        res.success = True
        return res

    def servo_m0_Cb(self, joint_data):
	self.servo = joint_data.data

def joint_transform(joint_d):
	# first: transform from j_d to j_m
	# T = [0.639, 0, 0
	#	Am_d]]
	j4 = joint_d[0]
	j5 = joint_d[1]
	j6 = joint_d[2]
	j7 = joint_d[3]
	
	T = np.array([[0.9817, 0, 0],[0.6695, 0.8211, -0.4106],[0.6695, 0.8211, -0.4106]])
	j_temp = np.array([j5, (0.5*j6+0.5*j7), (-j6+j7)])
	
	j_m5_7 = T.dot(j_temp)
	j_m4_7 = np.insert(j_m5_7,0,j4)
	
	joint_motor = test_msg()
	joint_motor = [0,0,0,0]

	
	# second: transform from j_m to actual motor pulses (0-1023)
	for i in range(len(j_m4_7)):
		if j_m4_7[i] >= 0.0:
			joint_motor[i] = 1023-195.64*(2.617-j_m4_7[i])
		else:
			joint_motor[i] = 195.64*(2.617+j_m4_7[i])

	return joint_motor

def main():
	n = rospy.init_node('servo_trajectory_controller')
    	servo_trajectory_controller = ServoTrajectoryController(n)
    	
	joint_data = test_msg()
	joint_data_motor = test_msg()

	msg = Int16MultiArray()
	trajectory = servo_trajectory_controller.traj

	msg.data = []
    
    	r = rospy.Rate(50) # 50 Hz
    	while not rospy.is_shutdown():
		print("servo_trajectory_controller is running!")
		
		trajectory = servo_trajectory_controller.traj		
		
		if len(trajectory) != 0:		
			servo_trajectory_controller.count
			
			joint_data.data = trajectory[servo_trajectory_controller.count].positions
			
			print(joint_data.data)
			
			joint_data_motor.data = joint_transform(joint_data.data)
						
			print(joint_data_motor.data)

			msg = Int16MultiArray(data=joint_data_motor.data)

			servo_trajectory_controller.servo_command_pub.publish(msg)
			

			servo_trajectory_controller.count+=1
			if servo_trajectory_controller.count >= len(trajectory):
				servo_trajectory_controller.traj = []
				servo_trajectory_controller.count = 0
			
		
		servo_joint_state = JointState()
		servo_joint_state.name = ['servo1', 'servo2', 'servo3', 'servo4']
		servo_joint_state.position = joint_data.data		
		servo_trajectory_controller.servo_joint_state_pub.publish(servo_joint_state)
		r.sleep()


if __name__ == '__main__':
	main()
