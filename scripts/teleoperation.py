#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from surgical_industrial_arm.srv import *
from geometry import *
from pyquaternion import Quaternion
import modern_robotics as mr



class Teleoperation:
    def __init__(self, n):
        self.n = n
        self.joint_state_subscriber = rospy.Subscriber('/dvrk/MTMR/position_cartesian_current', PoseStamped,
                                                       self.teleoperation_pose_cb)
        self.gripper_close_subscriber = rospy.Subscriber('/dvrk/MTMR/gripper_closed_event', Bool,
                                                         self.gripper_cloase_cb)
        self.position_offset = np.array([-0.179, -0.018, -0.259])
        self.position_scale = 0.3
        self.current_position = np.array([0, 0, 0])
        self.current_orientation = np.array([1, 0, 0, 0])
        self.p_rcm = np.array([0.402, 0, 0.405])
        self.z_tip_offset = 0.38

        self.q_original = Quaternion()
        self.master_pose_original = np.eye(4)
        self.master_home_pose = np.eye(4)
        self.is_master_pose_received = False

        self.gripper_close = False

        self.slave_home_pose = np.array([[0, 0, 1.0, 4.02e-01],
                                         [0.0, 1.0, 0.0, 0],
                                         [-1.0, 0.0, 0.0, 2.2e-02],
                                         [0.0, 0.0, 0.0, 1.0]])

    def teleoperation_pose_cb(self, data):
        self.is_master_pose_received = True
        #print("position: {}".format(data.pose.position))
        self.current_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.q_original = Quaternion(w=data.pose.orientation.w, x=data.pose.orientation.x,
                                y=data.pose.orientation.y, z=data.pose.orientation.z, )
        master_pose_original = matrix_from_pose_msg(data.pose)
        T_z90 = np.zeros((4,4))
        T_z90[0, 1] = 1
        T_z90[1, 0] = -1
        T_z90[2, 2] = 1
        T_z90[3, 3] = 1
        self.master_pose_original = np.matmul(mr.TransInv(T_z90), master_pose_original)
        # print("orientation: {}".format(q_original*q_rot))

    def gripper_cloase_cb(self, data):
        self.gripper_close = data.data

    def set_master_home_pose(self, pose):
        self.master_home_pose = pose


def main():
    n = rospy.init_node('teleoperation_node')
    teleoperation = Teleoperation(n)
    # wait for a little bit time for everything to be ready
    while not teleoperation.is_master_pose_received:
        print("Waiting for master pose message...")
        rospy.sleep(0.5)
    teleoperation.set_master_home_pose(teleoperation.master_pose_original)
    r = rospy.Rate(5) # 50 Hz
    cnt = 0
    rospy.wait_for_service('move_to_pose')

    while not rospy.is_shutdown():
        try:
            req = MoveToPoseRequest()
            #
            # q_rot1 = Quaternion(degrees=90, axis=(0.0, 0.0, 1.0))
            # q_rot2 = Quaternion(degrees=-90, axis=(1.0, 0.0, 0.0))
            #
            # r_gripper2tool = np.matrix([[0, 0, -1.0], [0, -1.0, 0], [-1.0, 0, 0]])
            # q_gripper2tool = Quaternion(matrix=r_gripper2tool)
            # # q_d = q_gripper2tool*teleoperation.q_original
            # print(teleoperation.q_original.rotation_matrix)
            # q_d = q_rot1*q_rot2*teleoperation.q_original
            # #q_d = q_rot1 * q_rot2 * teleoperation.q_original
            # #print(teleoperation.q_original.rotation_matrix)
            # #print(q_d[0], q_d[1], q_d[2], q_d[3])
            # q_home_tool = Quaternion(w=0.707, x=0.0, y =0.707, z=0.0)

            # print(teleoperation.master_pose_original)
            # T_master_current2home = np.matmul(teleoperation.master_pose_original, mr.TransInv(teleoperation.master_home_pose))
            master_R_original, master_p_original = mr.TransToRp(teleoperation.master_pose_original)
            master_R_home, master_p_home = mr.TransToRp(teleoperation.master_home_pose)
            master_p_current2home = master_p_original - master_p_home
            master_R_current2home = np.matmul(master_R_original, mr.RotInv(master_R_home))
            # print(master_R_current2home, master_p_current2home)
            # R_z_90 = np.zeros((3,3))
            # R_z_90[0, 1] = 1.0
            # R_z_90[1, 0] = -1.0
            # R_z_90[2, 2] = 1.0
            #
            # master_R_current2home = np.matmul(R_z_90, master_R_current2home)
            # master_p_current2home = np.matmul(R_z_90, master_p_current2home)

            slave_R_home, slave_p_home = mr.TransToRp(teleoperation.slave_home_pose)
            slave_R_des = np.matmul(master_R_current2home, slave_R_home)
            slave_p_des = master_p_current2home * teleoperation.position_scale + slave_p_home

            T_slave_des = mr.RpToTrans(slave_R_des, slave_p_des)
            #print(T_slave_des)
            slave_pose_msg = pose_msg_from_matrix(T_slave_des)
            print(slave_pose_msg)
            req.pose = slave_pose_msg

            # dis = Quaternion.distance(q_d, q_home_tool)
            # print(master_R_current2home[2, 2])
            # if master_R_current2home[2, 2] < 0.8:
            #     req.pose.orientation.w = 0.707#q_d[0]
            #     req.pose.orientation.x = 0.0#q_d[1]
            #     req.pose.orientation.y = 0.707#q_d[2]
            #     req.pose.orientation.z = 0.0#q_d[3]
            # # # print(dis)
            # # # print(q_d)
            # #
            # #
            # # # req.pose.orientation.w = q_d[0]
            # # # req.pose.orientation.x = q_d[1]
            # # # req.pose.orientation.y = q_d[2]
            # # # req.pose.orientation.z = q_d[3]
            # #
            # p_original = teleoperation.current_position - teleoperation.position_offset
            #
            # p_d = np.matmul(q_rot1.rotation_matrix, p_original) * teleoperation.position_scale + teleoperation.p_rcm
            # p_d[2] -= teleoperation.z_tip_offset
            #
            # req.pose.position.x = p_d[0]
            # req.pose.position.y = p_d[1]
            # req.pose.position.z = p_d[2]
            #
            if teleoperation.gripper_close:
                req.jaw_angle = 0
            else:
                req.jaw_angle = np.pi/3

            req.secs = 0.1
            # # print(req)
            #
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

        # r.sleep()


if __name__ == '__main__':
    main()