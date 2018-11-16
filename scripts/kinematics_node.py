#!/usr/bin/env python
from control_msgs.msg import JointTrajectoryActionGoal, JointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
#from MoveToJointAngle.srv import *
from surgical_industrial_arm.srv import *
import actionlib
import rospy
import random
import modern_robotics as mr
import numpy as np
from math import pi
import copy
from geometry import *
from pyquaternion import Quaternion



class Arm:
    def __init__(self, arm_name, n):

        self.tool_len = 0.416

        # frame space -> s
        # frame RCM -> r

        self.T_s_r = np.array([[1, 0, 0, 0.474], [0, 1, 0, 0], [0, 0, 1, 0.214], [0, 0, 0, 1]])
        # self.T_s_r = np.array([[1, 0, 0, 0.674], [0, 1, 0, 0], [0, 0, 1, 0.414], [0, 0, 0, 1]])
        self.M_s_e = np.array([[1, 0, 0, 0.474], [0, 1, 0, 0], [0, 0, 1, 0.63], [0, 0, 0, 1]])

        self.M_r_e = np.matmul(mr.TransInv(self.T_s_r), self.M_s_e)

        self.M_s_jaw = np.array([[1, 0, 0, 0.474], [0, 1, 0, 0], [0, 0, 1, 0.205], [0, 0, 0, 1]])
        self.M_r_jaw = np.matmul(mr.TransInv(self.T_s_r), self.M_s_jaw)

        self.S_s_jaw = np.array([[0, 0, 1, 0, 0, 0],
                          [0, 1, 0, -0.29, 0, 0],
                          [0, 1, 0, -0.56, 0, 0],
                          [1, 0, 0, 0, 0.63, 0],
                          [0, 1, 0, -0.63, 0, 0.302],
                          [1, 0, 0, 0, 0.63, 0],

                          [0, 0, -1, 0, 0.474, 0],
                          [0, 1, 0, -0.214, 0, 0.474],
                          [1, 0, 0, 0, 0.205, 0],
                          [1, 0, 0, 0, 0.205, 0]]).T

        self.S_s_e = np.array([[0, 0, 1, 0, 0, 0],
                          [0, 1, 0, -0.29, 0, 0],
                          [0, 1, 0, -0.56, 0, 0],
                          [1, 0, 0, 0, 0.63, 0],
                          [0, 1, 0, -0.63, 0, 0.302],
                          [1, 0, 0, 0, 0.63, 0]]).T

        self.S_r_jaw = np.array([[0, 1, 0, 0, 0, 0],
                          [1, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, -1],
                          [0, 0, -1, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0],
                          [-1, 0, 0, 0, -0.0091, 0]]).T

        self.S_r_e = np.array([[0, 1, 0, 0, 0, 0],
                          [1, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1]]).T

        self.robot_dof = 6
        self.tool_dof = 4
        self.intra_dof = 6
        self.intra_end_dof = 3
        self.joint_num = self.robot_dof + self.tool_dof
        self.joint_angle = np.array([0.0] * self.joint_num)
        self.motor_angle = np.array([0.0] * self.joint_num)
        self.intra_joint_angle = np.array([0.0] * self.intra_dof)
        self.intra_joint_angle[2] = 0.05
        self.intra_end_joint_angle = np.array([0.0] * self.intra_end_dof)
        self.intra_end_joint_angle[2] = -self.intra_joint_angle[2]
        self.joint_angle_received = False
        # arm_name should be l_arm or r_arm
        self.node = n
        self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        # move_jp, move_jr, move_cp, move_cr
        self.srv_move_to_joint_angle = rospy.Service('move_to_joint_angle', MoveToJointAngle,
                                                     self.move_to_joint_angle_handle)
        self.srv_move_to_pose = rospy.Service('move_to_pose', MoveToPose,
                                                     self.move_to_pose_handle)
        self.name = arm_name
        self.jta = actionlib.SimpleActionClient('/joint_trajectory_action', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')


    def move_to_joint_angle_handle(self, req):
        print "Receive Joint Angle"
        print req.angle
        self.move_to_joint_angle(req.angle, req.secs, 100)
        res = MoveToJointAngleResponse()
        res.success = True
        return res

    def move_to_pose_handle(self, req):
        print "Receive Pose in %s frame: [ x: %s, y: %s, z: %s, qx: %s, qy: %s, qz: %s, qw: %s]" %\
              (req.frame, req.pose.position.x, req.pose.position.y, req.pose.position.z, \
               req.pose.orientation.x,req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w)
        #return AddTwoIntsResponse(req.a + req.b)
        T = matrix_from_pose_msg(req.pose)
        self.move_to_pose(T, req.secs, 100, 'c', req.frame)
        res = MoveToPoseResponse()
        res.success = True
        return res

    def joint_angle2motor_angle(self, joint_angle):
        motor_angle = joint_angle
        motor_angle[self.joint_num - 2] = joint_angle[self.joint_num - 2] - 0.5 * joint_angle[self.joint_num - 1]
        motor_angle[self.joint_num - 1] = joint_angle[self.joint_num - 2] + 0.5 * joint_angle[self.joint_num - 1]

        return motor_angle


    def motor_angle2joint_angle(self, motor_angle):
        joint_angle = motor_angle
        joint_angle[self.joint_num - 2] = 0.5 * (motor_angle[self.joint_num-2] + motor_angle[self.joint_num-1])
        joint_angle[self.joint_num - 1] = -motor_angle[self.joint_num - 2] + motor_angle[self.joint_num - 1]

        return joint_angle

    def get_current_pos(self):
        while self.joint_angle_received == False:
            rospy.loginfo('Waiting for joint angles...')
            rospy.sleep(1)
        return self.forward_kinematics(self.joint_angle[:-1], frame='jaw', ref='s')


    def move_joint_traj(self, traj, dt):
        goal = FollowJointTrajectoryActionGoal()
        goal.goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8', 'joint_9', 'joint_a']
        for i in range(len(traj)):
            point = JointTrajectoryPoint()

            p = [0]*self.joint_num
            v = [0]*self.joint_num
            last_v = [0] * self.joint_num
            a = [0]*self.joint_num
            if i != len(traj)-1 and i != 0:
                for j in range(self.joint_num):
                    p[j] = traj[i][j]
                    v[j] = (traj[i+1][j] - traj[i][j])/dt
                    last_v[j] = (traj[i][j] - traj[i-1][j])/dt
                    a[j] = (v[j] - last_v[j])/dt
            if i == len(traj)-1:
                p = traj[-1]
            elif i == 0:
                p = traj[0]
            point.positions = p
            point.velocities = v
            point.accelerations = a
            point.time_from_start = rospy.Duration((i+1)*dt)
            goal.goal.trajectory.points.append(point)

        self.jta.send_goal_and_wait(goal.goal)


    def move_to_joint_angle(self, joint_angles, T, N):
        while self.joint_angle_received == False:
            rospy.loginfo('Waiting for joint angles...')
            rospy.sleep(1)

        motor_angles = self.joint_angle2motor_angle(joint_angles)
        print('joint_angles {}'.format(joint_angles))
        print('motor_angles {}'.format(motor_angles))

        traj = mr.JointTrajectory(self.joint_angle, motor_angles, T, N, 5)
        dt = T/(N-1.0)
        self.move_joint_traj(traj, dt)

    def move_to_pose(self, pose, Tf, N, inter_type, frame='jaw'):
        while self.joint_angle_received == False:
            rospy.loginfo('Waiting for joint angles...')
            rospy.sleep(1)

        # Cartisian or joint interplation
        if inter_type == 'c':
            x_start = self.get_current_pos()
            print('xstart', x_start)
            x_end = pose
            print('xend', x_end)
            x_traj = mr.CartesianTrajectory(x_start, x_end, Tf, N, 5)
            #print(x_traj)
            traj = []
            for i in range(len(x_traj)):
                if i == 0:
                    theta0 = self.joint_angle[:-1]
                else:
                    theta0 = traj[i-1]
                #print(x_traj[i])
                #print(theta0)
                theta = self.inverse_kinematics_rcm(x_traj[i])
                #print(theta)
                traj.append(theta)
        elif inter_type == 'j':
            theta_d = self.inverse_kinematics(pose, self.joint_angle, frame)
            traj = mr.JointTrajectory(self.joint_angle, theta_d, Tf, N, 5)
        dt = Tf/(N-1.0)
        self.move_joint_traj(traj, dt)


    def joint_state_callback(self, data):
        for i in range(self.joint_num):
            self.motor_angle[i] = data.position[i]
        self.joint_angle = self.motor_angle2joint_angle(self.motor_angle)
        self.joint_angle_received = True

    def forward_kinematics(self, angles, frame='jaw', ref='s'):
        if frame == 'jaw' and ref == 's':
            return mr.FKinSpace(self.M_s_jaw, self.S_s_jaw, angles)
        if frame == 'jaw' and ref == 'r':
            return mr.FKinSpace(self.M_r_jaw, self.S_r_jaw, angles)
        elif frame == 'end' and ref == 'r':
            return mr.FKinSpace(self.M_r_e, self.S_r_e, angles)
        else:
            raise Exception('Frame or reference is not right.')

    def inverse_kinematics(self, pose, thetalist0, frame='end', ref='s'):
        if frame == 'jaw' and ref == 's':
            return mr.IKinSpace(self.S_s_jaw, self.M_s_jaw, pose, thetalist0, 0.001, 0.0001)
        elif frame == 'end' and ref == 'r':
            return mr.IKinSpace(self.S_r_e, self.M_r_e, pose, thetalist0, 0.001, 0.0001)
        elif frame == 'jaw' and ref == 'r':
            return mr.IKinSpace(self.S_r_jaw, self.M_r_jaw, pose, thetalist0, 0.001, 0.0001)
        elif frame == 'end' and ref == 's':
            return mr.IKinSpace(self.S_s_e, self.M_s_e, pose, thetalist0, 0.001, 0.0001)
        else:
            raise Exception('Frame or reference is not right.')

    def inverse_kinematics_rcm(self, pose):
        pose_r_jaw = np.matmul(mr.TransInv(self.T_s_r), pose)
        # print("tar pose in space frame: {}".format(pose))
        # print("tar pose in rcm frame: {}".format(pose_r_jaw))
        # print("original intra joint angle: {}".format(self.intra_joint_angle))
        # print("original pose in rcm frame: {}".format(self.forward_kinematics(self.intra_joint_angle, 'jaw', 'r')))
        tar_intra_joint_angle, _ = self.inverse_kinematics(pose_r_jaw, self.intra_joint_angle, frame='jaw', ref='r')
        print("tar intra joint angle: {}".format(tar_intra_joint_angle))
        # print("original intra end joint angle: {}".format(self.intra_end_joint_angle))
        self.intra_end_joint_angle[0:2] = tar_intra_joint_angle[0:2]
        self.intra_end_joint_angle[2] = -tar_intra_joint_angle[2]
        # print("tar intra end joint angle: {}".format(self.intra_end_joint_angle))

        tar_pose_r_e = self.forward_kinematics(self.intra_end_joint_angle, frame='end', ref='r')
        tar_pose_s_e = np.matmul(self.T_s_r, tar_pose_r_e)
        # print("tar_pose_s_e: {}".format(tar_pose_s_e))
        tar_robot_joint_angle, _ = self.inverse_kinematics(tar_pose_s_e, self.joint_angle[:self.robot_dof], frame='end', ref='s')
        # print("tar_robot_joint_angle: {}".format(tar_robot_joint_angle))
        tar_joint_angle = self.joint_angle.copy()
        tar_joint_angle[:self.robot_dof] = np.array(tar_robot_joint_angle)
        tar_joint_angle[self.robot_dof:-1] = tar_intra_joint_angle[3:]
        # print("tar_joint_angle: {}".format(tar_joint_angle))

        return tar_joint_angle



def main():
    n = rospy.init_node('joint_position_tester')
    arm = Arm('r_arm', n)
    arm.move_to_joint_angle([0.0]*arm.joint_num, 3, 100)
    print("move to home position!")
    # rospy.sleep(3)
    # angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # arm.move_to_joint_angle(angles, 3, 100)
    # print("move to working home position!")
    # rospy.sleep(3)
    # print("current {} pose:".format('end'))
    # arm.intra_joint_angle[2] = 0.2
    # T = arm.forward_kinematics(arm.intra_joint_angle, 'jaw', 'r')
    # print(T)
    # T[1, 3] -= 0.01
    # print('pose: {}'.format(T))
    # # arm.move_to_pose(T, 2, 200, 'c')
    # print("original intra joint angles: {}".format(arm.intra_joint_angle))
    # ij, _ = arm.inverse_kinematics(T, arm.intra_joint_angle, frame='jaw', ref='r')
    # print("intra joint angles: {}".format(ij))

    T = arm.forward_kinematics(arm.joint_angle[:-1], 'jaw', 's')
    print('pose1: {}'.format(T))


    # print("original joint angles: {}".format(arm.joint_angle))
    # T[2, 3] -= 0.1
    # # T[0, 3] -= 0.01
    # print('pose2: {}'.format(T))
    # tar_joint_angle = arm.inverse_kinematics_rcm(T)
    # # print("target joint angles: {}".format(tar_joint_angle))
    # # angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # arm.move_to_joint_angle(tar_joint_angle, 3, 100)
    #
    #
    # print("original joint angles: {}".format(arm.joint_angle))
    # T[0, 3] -= 0.05
    # # T[0, 3] -= 0.01
    # print('pose2: {}'.format(T))
    # tar_joint_angle = arm.inverse_kinematics_rcm(T)
    # # print("target joint angles: {}".format(tar_joint_angle))
    # # angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # arm.move_to_joint_angle(tar_joint_angle, 3, 100)
    #
    #
    # print("original joint angles: {}".format(arm.joint_angle))
    # T[0, 3] += 0.1
    # # T[0, 3] -= 0.01
    # print('pose2: {}'.format(T))
    # tar_joint_angle = arm.inverse_kinematics_rcm(T)
    # # print("target joint angles: {}".format(tar_joint_angle))
    # # angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # arm.move_to_joint_angle(tar_joint_angle, 3, 100)
    #
    #
    # print("original joint angles: {}".format(arm.joint_angle))
    # T[0, 3] -= 0.05
    # T[1, 3] -= 0.05
    # # T[0, 3] -= 0.01
    # print('pose2: {}'.format(T))
    # tar_joint_angle = arm.inverse_kinematics_rcm(T)
    # # print("target joint angles: {}".format(tar_joint_angle))
    # # angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # arm.move_to_joint_angle(tar_joint_angle, 3, 100)
    #
    # print("original joint angles: {}".format(arm.joint_angle))
    # T[1, 3] += 0.05
    # # T[0, 3] -= 0.01
    # print('pose2: {}'.format(T))
    # tar_joint_angle = arm.inverse_kinematics_rcm(T)
    # # print("target joint angles: {}".format(tar_joint_angle))
    # # angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # arm.move_to_joint_angle(tar_joint_angle, 3, 100)

    T[2, 3] -= 0.15
    arm.move_to_pose(T, 2, 50, 'c')

    T[0, 3] -= 0.05
    arm.move_to_pose(T, 2, 50, 'c')


    T[0, 3] += 0.05
    arm.move_to_pose(T, 2, 50, 'c')

    T[0, 3] += 0.05
    arm.move_to_pose(T, 2, 50, 'c')

    T[0, 3] -= 0.05
    T[1, 3] -= 0.05
    arm.move_to_pose(T, 2, 50, 'c')

    T[0, 3] -= 0.05
    T[1, 3] += 0.05
    arm.move_to_pose(T, 2, 50, 'c')


    rospy.spin()


if __name__ == '__main__':
    main()
