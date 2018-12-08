#!/usr/bin/env python
from control_msgs.msg import JointTrajectoryActionGoal, JointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
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
import sys



class Arm:
    def __init__(self, arm_name, n, is_simulation):
        self.tool_len = 0.416
        self.T_s_r = None
        self.M_r_e = None
        self.M_r_jaw = None

        self.is_sim = is_simulation

        # frame space -> s
        # frame RCM -> r

        self.robot_dof = 6
        self.tool_dof = 3
        self.jaw_dof = 1
        self.intra_dof = 6
        self.intra_end_dof = 4
        self.joint_num = self.robot_dof + self.tool_dof
        self.motor_num = self.joint_num + 1

        self.jaw_len = 0.01

        self.M_s_e = np.array([[1, 0, 0, 0.89], [0, 1, 0, 0], [0, 0, 1, 0.73], [0, 0, 0, 1]])
        self.M_s_jaw = np.array([[1, 0, 0, 0.900 + self.jaw_len], [0, 1, 0, 0], [0, 0, 1, 0.73], [0, 0, 0, 1]])
        p_rcm = np.array([0.402, 0, 0.405])
        self.set_rcm(p_rcm)

        self.S_s_jaw = np.array([[0, 0, 1, 0, 0, 0],
                          [0, 1, 0, -0.29, 0, 0],
                          [0, 1, 0, -0.56, 0, 0],
                          [1, 0, 0, 0, 0.63, 0],
                          [0, 1, 0, -0.63, 0, 0.302],
                          [1, 0, 0, 0, 0.63, 0],

                          [1, 0, 0, 0, 0.73, 0],
                          [0, 1, 0, -0.73, 0, 0.89],
                          [0, 0, 1, 0, -0.9, 0]]).T

        self.S_s_e = self.S_s_jaw[:, :self.robot_dof]

        self.S_r_jaw = np.array([[0, 1, 0, 0, 0, 0],
                          [1, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, -1],
                          [0, 0, -1, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0],
                          [1, 0, 0, 0, 0.0091, 0]]).T

        self.S_r_e = self.S_r_jaw[:, :4]

        self.joint_angle = np.array([0.0] * self.joint_num)
        self.last_joint_angle = self.joint_angle.copy() # for interpolation
        self.motor_angle = np.array([0.0] * (self.joint_num + 1))
        self.jaw_angle = 0
        self.tar_jaw_angle = 0
        self.intra_joint_angle = np.array([0.0] * self.intra_dof)
        self.last_intra_joint_angle = self.intra_joint_angle.copy()
        self.intra_joint_angle[2] = 0.2
        self.intra_end_joint_angle = self.intra_joint_angle[:self.intra_end_dof]
        self.last_robot_joint_angle = self.joint_angle[:self.robot_dof]
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

        if not self.is_sim:
            rospy.loginfo("waiting for servo_trajectory service...")
            rospy.wait_for_service('servo_trajectory')
            req = ServoTrajectoryRequest()

            point = JointTrajectoryPoint()
            point.positions = [0, 0, 0, 0]
            point.time_from_start = rospy.Duration(3)

            req.points.append(point)
            req.points.append(point)
            req.points.append(point)

            servo_trajectory = rospy.ServiceProxy('servo_trajectory', ServoTrajectory)
            res = servo_trajectory(req)
            rospy.loginfo('Found servo joint trajectory service!')

        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def set_rcm(self, p):
        self.T_s_r = np.eye(4)
        self.T_s_r[0, 3] = p[0]
        self.T_s_r[1, 3] = p[1]
        self.T_s_r[2, 3] = p[2]


        self.M_r_e = np.zeros((4, 4))
        self.M_r_e[0, 2] = 1
        self.M_r_e[1, 1] = 1
        self.M_r_e[2, 0] = -1
        self.M_r_e[3, 3] = 1

        self.M_r_jaw = self.M_r_e.copy()
        self.M_r_jaw[2, 3] = -0.0091 - self.jaw_len

        print("M_r_e: {}".format(self.M_r_e))
        print("M_r_jaw: {}".format(self.M_r_jaw))

    def move_to_joint_angle_handle(self, req):
        print "Receive Joint Angle"
        print req.angle
        self.move_to_joint_angle(req.angle[:9], req.angle[9], req.secs, 100)
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

    def joint_angle2motor_angle(self, joint_angle, jaw_angle):
        motor_angle = np.zeros(self.motor_num)
        motor_angle[:-2] = joint_angle[:-1]
        motor_angle[-2] = joint_angle[-1] - 0.5 * jaw_angle
        motor_angle[-1] = joint_angle[-1] + 0.5 * jaw_angle
        return motor_angle

    def motor_angle2joint_angle(self, motor_angle):
        joint_angle = np.zeros(self.joint_num)
        joint_angle[:-1] = motor_angle[:-2]
        joint_angle[-1] = 0.5 * (motor_angle[-2] + motor_angle[-1])
        jaw_angle = motor_angle[-1] - motor_angle[-2]

        return joint_angle, jaw_angle

    def get_current_pos(self):
        while self.joint_angle_received == False:
            rospy.loginfo('Waiting for joint angles...')
            rospy.sleep(1)
        return self.forward_kinematics(self.joint_angle, frame='jaw', ref='s')

    def move_joint_traj(self, traj, dt):
        goal = FollowJointTrajectoryActionGoal()
        goal.goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8', 'joint_9', 'joint_a']
        # for real robot, only send 6 joints
        if not self.is_sim:
            goal.goal.trajectory.joint_names = goal.goal.trajectory.joint_names[:self.robot_dof]

        for i in range(len(traj)):
            point = JointTrajectoryPoint()

            p = [0]*self.motor_num
            v = [0]*self.motor_num
            last_v = [0] * self.motor_num
            a = [0]*self.motor_num
            if i != len(traj)-1 and i != 0:
                for j in range(self.motor_num):
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
        rospy.sleep(0.5)

    def move_to_joint_angle(self, joint_angles, jaw_angle, T, N):
        while self.joint_angle_received == False:
            rospy.loginfo('Waiting for joint angles...')
            rospy.sleep(1)

        self.tar_jaw_angle = jaw_angle
        motor_angle_d = self.joint_angle2motor_angle(joint_angles, jaw_angle)
        # print('current_motor_angles {}'.format(self.motor_angle))
        # print('desired_motor_angles {}'.format(motor_angle_d))

        traj = mr.JointTrajectory(self.motor_angle, motor_angle_d, T, N, 5)
        dt = T/(N-1.0)
        self.move_joint_traj(traj, dt)

    def move_to_pose(self, pose, Tf, N, inter_type, frame='jaw'):
        while self.joint_angle_received == False:
            rospy.loginfo('Waiting for joint angles...')
            rospy.sleep(1)
        traj = []
        # self.set_last_tar_joint_angles2cur()
        # Cartisian or joint interplation
        if inter_type == 'c':
            x_start = self.get_current_pos()
            print('xstart', x_start)
            x_end = pose
            print('xend', x_end)
            x_traj = mr.CartesianTrajectory(x_start, x_end, Tf, N, 5)
            #print(x_traj)

            for i in range(len(x_traj)):
                if i == 0:
                    theta0 = self.joint_angle[:-1]
                else:
                    theta0 = traj[i-1]
                #print(x_traj[i])
                #print(theta0)
                theta_d = self.inverse_kinematics_rcm(x_traj[i])
                motor_angle_d = self.joint_angle2motor_angle(theta_d, self.tar_jaw_angle)
                #print(theta)
                traj.append(motor_angle_d)
        elif inter_type == 'j':
            theta_d = self.inverse_kinematics_rcm(pose)
            print("theta_d: {}".format(theta_d))
            motor_angle_d = self.joint_angle2motor_angle(theta_d, self.tar_jaw_angle)
            print("motor_d: {}".format(motor_angle_d))
            print("tar motor angle: {}".format(motor_angle_d))
            traj = mr.JointTrajectory(self.motor_angle, motor_angle_d, Tf, N, 5)
        dt = Tf/(N-1.0)
        self.move_joint_traj(traj, dt)

    def joint_state_callback(self, data):
        for i in range(self.motor_num):
            self.motor_angle[i] = data.position[i]
        self.joint_angle, self.jaw_angle = self.motor_angle2joint_angle(self.motor_angle)
        self.intra_joint_angle[-2:] = self.joint_angle[-2:]
        self.joint_angle_received = True

    def forward_kinematics(self, angles, frame='jaw', ref='s'):
        if frame == 'jaw' and ref == 's':
            return mr.FKinSpace(self.M_s_jaw, self.S_s_jaw, angles)
        if frame == 'jaw' and ref == 'r':
            return mr.FKinSpace(self.M_r_jaw, self.S_r_jaw, angles)
        elif frame == 'end' and ref == 'r':
            return mr.FKinSpace(self.M_r_e, self.S_r_e, angles)
        elif frame == 'end' and ref == 's':
            return mr.FKinSpace(self.M_s_e, self.S_s_e, angles)
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
        # print("tar jaw in RCM frame: {}".format(pose_r_jaw))
        # print("cur intra joint angle: {}".format(self.intra_joint_angle))
        # print("fk for cur intra joint angle in RCM frame: {}".format(
        #     self.forward_kinematics(self.intra_joint_angle, 'jaw', 'r')))
        tar_intra_joint_angle, err_intra_ik = self.inverse_kinematics(pose_r_jaw, self.intra_joint_angle, frame='jaw', ref='r')
        self.intra_joint_angle = tar_intra_joint_angle.copy()
        # print("tar intra joint angle: {}".format(tar_intra_joint_angle))
        # print("fk for tar intra joint angle in RCM frame: {}".format(self.forward_kinematics(tar_intra_joint_angle, 'jaw', 'r')))
        # print("intra IK succ: {}".format(err_intra_ik))
        # print("original intra end joint angle: {}".format(self.intra_end_joint_angle))
        self.intra_end_joint_angle[0:4] = tar_intra_joint_angle[0:4]
        # print("tar intra end joint angle: {}".format(self.intra_end_joint_angle))

        tar_pose_r_e = self.forward_kinematics(self.intra_end_joint_angle, frame='end', ref='r')
        tar_pose_s_e = np.matmul(self.T_s_r, tar_pose_r_e)
        # print("tar_pose_s_e: {}".format(tar_pose_s_e))
        tar_robot_joint_angle, _ = self.inverse_kinematics(tar_pose_s_e, self.joint_angle[:self.robot_dof], frame='end', ref='s')
        # print("tar_robot_joint_angle: {}".format(tar_robot_joint_angle))
        tar_joint_angle = self.joint_angle.copy()
        tar_joint_angle[:self.robot_dof] = np.array(tar_robot_joint_angle)
        tar_joint_angle[-2:] = tar_intra_joint_angle[-2:]
        # print("tar_joint_angle: {}".format(tar_joint_angle))

        # self.last_joint_angle = tar_joint_angle
        # self.last_intra_joint_angle = tar_intra_joint_angle
        # self.last_intra_end_joint_angle = tar_robot_joint_angle

        return tar_joint_angle


def main(argv):
    n = rospy.init_node('joint_position_tester')
    is_simulation = True
    if rospy.has_param('~is_simulation'):
        is_simulation = rospy.get_param("~is_simulation", "true")
        rospy.logwarn("Kinematics Node is in " + ("Simulation Mode" if is_simulation else "Real Robot Mode"))
    else:
        rospy.logwarn("No argument input to indicate simulation or real robot.")

    arm = Arm('r_arm', n, is_simulation)
    joint_angle_d = [0.0]*arm.joint_num
    joint_angle_d[4] = 1.5707
    arm.move_to_joint_angle(joint_angle_d, 0.5, 3, 100)
    print("move to home position!")
    rospy.sleep(2)
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

    T = arm.forward_kinematics(arm.joint_angle, 'jaw', 's')
    print('jaw pose: {}'.format(T))
    print('jaw quaternion: {}'.format(Quaternion(matrix=T)))

    T = arm.forward_kinematics(arm.joint_angle[:arm.robot_dof], 'end', 's')
    print('end pose: {}'.format(T))
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
    T[2, 3] += 0.05
    # T[1, 3] -= 0.05
    arm.move_to_pose(T, 2, 50, 'c')
    rospy.sleep(0.5)

    # print("send goal")
    # goal = FollowJointTrajectoryActionGoal()
    # arm.jta_servo.send_goal_and_wait(goal)
    # print("goal sent")

    # T[0, 3] -= 0.03
    # arm.move_to_pose(T, 2, 50, 'c')
    # rospy.sleep(0.5)
    #
    # T[0, 3] += 0.06
    # arm.move_to_pose(T, 2, 50, 'c')
    #
    # T[0, 3] += 0.05
    # arm.move_to_pose(T, 2, 50, 'j')
    #
    # T[0, 3] -= 0.05
    # T[1, 3] -= 0.05
    # arm.move_to_pose(T, 2, 50, 'c')
    #
    # T[0, 3] -= 0.05
    # T[1, 3] += 0.05
    # arm.move_to_pose(T, 2, 50, 'c')


    rospy.spin()


if __name__ == '__main__':

    main(sys.argv)
