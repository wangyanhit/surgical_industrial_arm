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

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from surgical_industrial_arm.srv import ServoTrajectory, ServoTrajectoryRequest, ServoTrajectoryResponse


class ServoTrajectoryController:
    def __init__(self, n):
        self.node = n
        self.srv_servo_trajectory = rospy.Service('servo_trajectory', ServoTrajectory, self.servo_trajectory_cb)

    def servo_trajectory_cb(self, req):
        # success = True
        # fb = True
        print("Received servo action message: {}".format(req))
        # if success:
        #     self.jta_servo.publish_feedback(fb)
        res = ServoTrajectoryResponse()
        res.success = True
        return res


def main():
    n = rospy.init_node('servo_trajectory_controller')
    servo_trajectory_controller = ServoTrajectoryController(n)

    r = rospy.Rate(1) # 50 Hz
    while not rospy.is_shutdown():
        print("servo_trajectory_controller is running!")
        r.sleep()


if __name__ == '__main__':
    main()