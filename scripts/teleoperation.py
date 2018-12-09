#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from surgical_industrial_arm.srv import *
from geometry import *
from pyquaternion import Quaternion


class Teleoperation:
    def __init__(self, n):
        self.n = n
        self.joint_state_subscriber = rospy.Subscriber('/dvrk/MTMR/position_cartesian_current', PoseStamped,
                                                       self.teleoperation_pose_cb)
        self.gripper_close_subscriber = rospy.Subscriber('/dvrk/MTMR/gripper_closed_event', Bool,
                                                         self.gripper_cloase_cb)
        self.position_offset = np.array([-0.179, -0.018, -0.259])
        self.position_scale = 0.2
        self.current_position = np.array([0, 0, 0])
        self.current_orientation = np.array([1, 0, 0, 0])
        self.p_rcm = np.array([0.402, 0, 0.405])
        self.z_tip_offset = 0.38

        self.q_original = Quaternion()

        self.gripper_close = False

    def teleoperation_pose_cb(self, data):
        #print("position: {}".format(data.pose.position))
        self.current_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.q_original = Quaternion(w=data.pose.orientation.w, x=data.pose.orientation.x,
                                y=data.pose.orientation.y, z=data.pose.orientation.z, )
        # print("orientation: {}".format(q_original*q_rot))

    def gripper_cloase_cb(self, data):
        self.gripper_close = data.data


def main():
    n = rospy.init_node('teleoperation_node')
    teleoperation = Teleoperation(n)
    r = rospy.Rate(1) # 50 Hz
    cnt = 0
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service('move_to_pose')

            req = MoveToPoseRequest()

            q_rot1 = Quaternion(degrees=90, axis=(0.0, 0.0, 1.0))
            q_rot2 = Quaternion(degrees=-90, axis=(1.0, 0.0, 0.0))

            r_gripper2tool = np.matrix([[0, 0, -1.0], [0, -1.0, 0], [-1.0, 0, 0]])
            q_gripper2tool = Quaternion(matrix=r_gripper2tool)
            # q_d = q_gripper2tool*teleoperation.q_original
            print(teleoperation.q_original.rotation_matrix)
            q_d = q_rot1*q_rot2*teleoperation.q_original
            #q_d = q_rot1 * q_rot2 * teleoperation.q_original
            #print(teleoperation.q_original.rotation_matrix)
            #print(q_d[0], q_d[1], q_d[2], q_d[3])
            q_home_tool = Quaternion(w=0.707, x=0.0, y =0.707, z=0.0)
            dis = Quaternion.distance(q_d, q_home_tool)
            if dis < 0.5:
                req.pose.orientation.w = q_d[0]
                req.pose.orientation.x = q_d[1]
                req.pose.orientation.y = q_d[2]
                req.pose.orientation.z = q_d[3]
            else:
                req.pose.orientation.w = 0.707#q_d[0]
                req.pose.orientation.x = 0.0#q_d[1]
                req.pose.orientation.y = 0.707#q_d[2]
                req.pose.orientation.z = 0.0#q_d[3]
            # print(dis)
            # print(q_d)


            # req.pose.orientation.w = q_d[0]
            # req.pose.orientation.x = q_d[1]
            # req.pose.orientation.y = q_d[2]
            # req.pose.orientation.z = q_d[3]

            p_original = teleoperation.current_position - teleoperation.position_offset

            p_d = np.matmul(q_rot1.rotation_matrix, p_original) * teleoperation.position_scale + teleoperation.p_rcm
            p_d[2] -= teleoperation.z_tip_offset

            req.pose.position.x = p_d[0]
            req.pose.position.y = p_d[1]
            req.pose.position.z = p_d[2]

            if teleoperation.gripper_close:
                req.jaw_angle = 0
            else:
                req.jaw_angle = np.pi/3

            req.secs = 0.1
            # print(req)

            move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)
            res = move_to_pose(req)
            # if res.success:
            #     print('Successfully moved to the area above pad')
            # else:
            #     print('Failed to move to the area above pad')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        cnt += 1
        print(cnt)
        rospy.sleep(0.01)
        r.sleep()


if __name__ == '__main__':
    main()