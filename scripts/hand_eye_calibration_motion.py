#!/usr/bin/env python
from geometry_msgs.msg import Pose
import rospy
from math import pi
from pyquaternion import Quaternion
import numpy as np
from abb_control.srv import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry import pose_msg_from_matrix


class HandEyeCalibrationMotion:
    def __init__(self, n):
        self.node = n
        self.pose_pub = rospy.Publisher("/msrCartPos", Pose, queue_size=10)
        #self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        # position of the center of the checker boadrd
        self.c_x = 0.0
        self.c_y = 0.3
        self.c_z = 0

        self.r = 0.5

        self.angles = [10.0/180.0*pi, 15.0/180.0*pi]

        self.home_q = Quaternion(0, 0, 1, 0)
        self.work_home_q = Quaternion(0, -0.7071068, 0.7071068, 0)
        #self.joint_position = [0]*6

    def move(self):
        rospy.wait_for_service('move_to_pose')

        # iterate for each angle
        x_scales = [-1, 0, 1]
        y_scales = [-1, 0, 1]
        axis = np.array([0, 0, 0])
        p_z = np.array([0, 0, self.r])
        for angle in self.angles:
            print('angle %s' % angle)
            for x_s in x_scales:
                for y_s in y_scales:
                    angle_use = 0
                    if x_s == 0 and y_s == 0:
                        axis[0] = 1
                        axis[1] = 1
                    else:
                        axis[0] = x_s
                        axis[1] = y_s
                        axis = axis / np.linalg.norm(axis)
                        angle_use = angle
                    print('axis:')
                    print(axis)
                    q = Quaternion(axis=[axis[0], axis[1], axis[2]], angle=angle_use)
                    R = q.rotation_matrix
                    #print('R:')
                    #print(R)
                    p = np.dot(R, p_z) + np.array([self.c_x, self.c_y, self.c_z])
                    print('p %s', p)
                    q = q * self.work_home_q

                    try:
                        move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)
                        req = MoveToPoseRequest()
                        req.pose.position.x = p[0]
                        req.pose.position.y = p[1]
                        req.pose.position.z = p[2]
                        req.pose.orientation.w = q[0]
                        req.pose.orientation.x = q[1]
                        req.pose.orientation.y = q[2]
                        req.pose.orientation.z = q[3]

                        req.secs = 2.0
                        req.frame = 'end'

                        res = move_to_pose(req)
                        if res.success:
                            print('Successfully moved to given pose')
                            rospy.sleep(0.5)
                            self.pose_pub.publish(req.pose)
                        else:
                            print('Failed to move to given pose')
                    except rospy.ServiceException, e:
                        print "Service call failed: %s" % e
                    rospy.sleep(3)


def main():
    n = rospy.init_node('hand_eye_calibration_motion')
    calibration = HandEyeCalibrationMotion(n)
    calibration.move()
    rospy.spin()


if __name__ == '__main__':
    main()