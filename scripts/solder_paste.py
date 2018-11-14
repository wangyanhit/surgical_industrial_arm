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
from gerber_import.srv import *


class SolderPaste:
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

        self.pcb_pads_info = []
        self.pcb_pose = None
        # for test
        self.pcb_pose = [[1, 0, 0, 0], [0, 1, 0, 0.302], [0, 0, 1, 0.558], [0, 0, 0, 1]]
        '''
        self.pad_positions = []
        p = [0.01, 0.01]
        self.pad_positions.append(p)
        p = [0.01, 0.02]
        self.pad_positions.append(p)
        p = [0.01, 0.03]
        self.pad_positions.append(p)
        p = [0.02, 0.01]
        self.pad_positions.append(p)
        p = [0.02, 0.02]
        self.pad_positions.append(p)
        p = [0.02, 0.03]
        self.pad_positions.append(p)
        '''

        self.paste_duration = 0.5
        self.paste_z = 0.301
        self.move_z = 0.305

        self.request_mask_info()

    def paste_once(self, p, dt, first_time=True):
        rospy.wait_for_service('move_to_pose')
        # 1. move to the area above pad
        req = MoveToPoseRequest()
        req.pose.position.x = p[0]
        req.pose.position.y = p[1]
        req.pose.position.z = self.move_z
        req.pose.orientation.w = self.work_home_q[0]
        req.pose.orientation.x = self.work_home_q[1]
        req.pose.orientation.y = self.work_home_q[2]
        req.pose.orientation.z = self.work_home_q[3]
        if first_time:
            req.secs = 3
        else:
            req.secs = 0.5
        req.frame = 'end'
        try:
            move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)

            res = move_to_pose(req)
            if res.success:
                print('Successfully moved to the area above pad')
                rospy.sleep(0.5)
                self.pose_pub.publish(req.pose)
            else:
                print('Failed to move to the area above pad')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        rospy.sleep(1)

        # 2. move to pad
        req.pose.position.z = self.paste_z
        req.secs = 0.5
        try:
            move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)

            res = move_to_pose(req)
            if res.success:
                print('Successfully moved to pad')
                rospy.sleep(0.5)
                self.pose_pub.publish(req.pose)
            else:
                print('Failed to move to pad')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        rospy.sleep(1)


        # 3. paste
        rospy.loginfo('Pasting...')
        rospy.sleep(1)

        # 4. move up
        rospy.loginfo('Moving up...')
        req.pose.position.z = self.move_z
        req.secs = 0.5
        try:
            move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)

            res = move_to_pose(req)
            if res.success:
                print('Successfully moved up')
                rospy.sleep(0.5)
                self.pose_pub.publish(req.pose)
            else:
                print('Failed to move up')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def paste(self):
        for pad_info in self.pcb_pads_info:
            x = pad_info[0]
            y = pad_info[1]
            a = pad_info[2]
            dt = 1.0*a
            pad_p_in_pcb = np.array([x, y, self.paste_z, 1])
            pad_p_in_world = np.dot(self.pcb_pose, pad_p_in_pcb)
            print('pad postion in pcb is: {}'.format(pad_p_in_pcb[:3]))
            print('pad postion in world is: {}'.format(pad_p_in_world[:3]))
            if pad_info == self.pcb_pads_info[0]:
                self.paste_once(pad_p_in_world[:3], dt, True)
            else:
                self.paste_once(pad_p_in_world[:3], dt, False)

    def request_mask_info(self):
        rospy.wait_for_service('/gerber_import')
        try:
            gerber_mask_handle = rospy.ServiceProxy('/gerber_import', read_solder_mask)
            resp = gerber_mask_handle('/tmp/test.grb')
            #resp.mask
            #m = read_solder_maskResponse()
            num = resp.mask.solder_pads.size()
            for i in range(num):
                x = resp.mask.solder_pads[i].x_offset
                y = resp.mask.solder_pads[i].y_offset
                a = resp.mask.solder_pads[i].area

                self.pcb_pads_info.append([x, y, a])
            print "Get {} pads' info successfully".format(num)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

def main():
    n = rospy.init_node('solder_paste')
    solder_paste = SolderPaste(n)
    solder_paste.paste()
    rospy.spin()


if __name__ == '__main__':
    main()