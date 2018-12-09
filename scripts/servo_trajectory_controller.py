#!/usr/bin/env python
import rospy
import actionlib
import random
import modern_robotics as mr
import numpy as np
from math import pi
import copy
from control_msgs.msg import JointTrajectoryActionGoal, JointTrajectoryAction, FollowJointTrajectoryActionGoal,\
    FollowJointTrajectoryAction, FollowJointTrajectoryFeedback
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16, Int16MultiArray

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from surgical_industrial_arm.srv import ServoTrajectory, ServoTrajectoryRequest, ServoTrajectoryResponse


class ServoTrajectoryController:
    def __init__(self, n):
        self.node = n
        self.srv_servo_trajectory = rospy.Service('servo_trajectory', ServoTrajectory, self.servo_trajectory_cb)
        self.servo_joint_state_pub = rospy.Publisher('/servo_joint_states', JointState, queue_size=10)
        self.servo_command_pub = rospy.Publisher('ServoPoseSerial', Int16MultiArray, queue_size=10)
        self.motor_trajectory = []
        self.current_motor_angle = [0, 0, 0, 0]

        self.current_trajectory_point_cnt = 0

    def servo_trajectory_cb(self, req):
        # success = True
        # fb = True
        self.motor_trajectory = req.points
        print("Received servo action message: {}".format(self.motor_trajectory))
        # if success:
        #     self.jta_servo.publish_feedback(fb)
        res = ServoTrajectoryResponse()
        res.success = True
        return res

    def value_limit(self, value, min, max):
        if value < min:
            value = min
        elif value > max:
            value = max
        return value

    def motor_spcae2servo_space(self, motor_angle):
        servo_angle = [0, 0, 0, 0]
        motor_angle[0] = self.value_limit(motor_angle[0], np.deg2rad(-180), np.deg2rad(180))
        servo_angle[0] = motor_angle[0] * (0.6397)

        motor_angle[1] = self.value_limit(motor_angle[1], np.deg2rad(-90), np.deg2rad(90))
        servo_angle[1] = motor_angle[1] * -0.9817

        motor_angle[2] = self.value_limit(motor_angle[2], np.deg2rad(-90), np.deg2rad(90))
        motor_angle[3] = self.value_limit(motor_angle[3], np.deg2rad(-90), np.deg2rad(90))
        servo_angle[2] = motor_angle[1] * -0.6696 + motor_angle[2] * 0.8212
        servo_angle[3] = motor_angle[1] * -0.6696 + motor_angle[3] * 0.8212

        servo_angle[0] = self.value_limit(servo_angle[0], np.deg2rad(-150), np.deg2rad(150))
        servo_angle[1] = self.value_limit(servo_angle[1], np.deg2rad(-150), np.deg2rad(150))
        servo_angle[2] = self.value_limit(servo_angle[2], np.deg2rad(-150), np.deg2rad(150))
        servo_angle[3] = self.value_limit(servo_angle[3], np.deg2rad(-150), np.deg2rad(150))
        return servo_angle

    def pub_servo_angle(self, servo_angle):
        # remap servo order
        servo_angle_remap = [0, 0, 0, 0]
        servo_angle_remap[2] = servo_angle[0]
        servo_angle_remap[1] = servo_angle[1]
        servo_angle_remap[0] = servo_angle[2]
        servo_angle_remap[3] = servo_angle[3]

        cmd = Int16MultiArray()
        cmd.data = (np.array(servo_angle_remap) + np.deg2rad(150))/np.deg2rad(300)*1023

        self.servo_command_pub.publish(cmd)

    def pub_motor_angle(self, motor_angle):
        servo_angle = self.motor_spcae2servo_space(motor_angle)
        self.pub_servo_angle(servo_angle)

    def pub_motor_state(self, motor_angle):
        servo_joint_state = JointState()
        servo_joint_state.name = ['servo1', 'servo2', 'servo3', 'servo4']
        servo_joint_state.position = motor_angle
        self.servo_joint_state_pub.publish(servo_joint_state)


def main():
    n = rospy.init_node('servo_trajectory_controller')
    servo_trajectory_controller = ServoTrajectoryController(n)
    rospy.sleep(0.1)
    servo_trajectory_controller.pub_motor_angle([0, 0, 0, 0])
    rospy.sleep(0.1)

    r = rospy.Rate(20) # 20 Hz
    while not rospy.is_shutdown():
        print("servo_trajectory_controller is running!")
        servo_trajectory_controller.pub_motor_state(servo_trajectory_controller.current_motor_angle)

        if len(servo_trajectory_controller.motor_trajectory) > 0:
            if servo_trajectory_controller.current_trajectory_point_cnt < len(servo_trajectory_controller.motor_trajectory):
                servo_trajectory_controller.current_motor_angle =\
                    list(servo_trajectory_controller.motor_trajectory[servo_trajectory_controller.current_trajectory_point_cnt].positions)
                servo_trajectory_controller.pub_motor_angle(servo_trajectory_controller.current_motor_angle)
                servo_trajectory_controller.current_trajectory_point_cnt += 1
            else:
                servo_trajectory_controller.motor_trajectory = []
                servo_trajectory_controller.current_trajectory_point_cnt = 0

        r.sleep()


if __name__ == '__main__':
    main()