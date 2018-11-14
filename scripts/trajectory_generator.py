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


'''
M = [[0, 0, 1, 0.374], [0, 1, 0, 0], [-1, 0, 0, 0.63], [0, 0, 0, 1]]
Slist = np.array([[0, 0, 1, 0, 0, 0],
                  [0, 1, 0, -0.29, 0, 0],
                  [0, 1, 0, -0.56, 0, 0],
                  [1, 0, 0, 0, 0.63, 0],
                  [0, 1, 0, -0.63, 0, 0.302],
                  [1, 0, 0, 0, 0.63, 0]]).T
thetalist = [30.0/180*pi, 30.0/180*pi, 0, 30.0/180*pi, 30.0/180*pi, 30.0/180*pi]
thetalist0 = [0.0/180*pi, 0.0/180*pi, 0, 0.0/180*pi, 0.0/180*pi, 0.0/180*pi]
print(thetalist)
T = mr.FKinSpace(M, Slist, thetalist)
print(T)
thetalist1 = mr.IKinSpace(Slist, M, T,  thetalist0, 0.001, 0.0001)
print(thetalist1)

traj = mr.JointTrajectory(thetalist0, thetalist, 5, 500, 5)
print traj
'''



class Arm:
    def __init__(self, arm_name, n):

        needle_len = 0.25
        gripper_len = 0.2
        hot_air_len = 0.373
        self.M_end = [[0, 0, 1, 0.474], [-1, 0, 0, 0], [0, -1, 0, 0.63], [0, 0, 0, 1]]
        self.M_tool_top = [[1, 0, 0, 0.374], [0, 1, 0, 0], [0, 0, 1, 0.63], [0, 0, 0, 1]]
        self.M_needle = [[0, 0, 1, 0.374 + needle_len], [0, 1, 0, 0], [-1, 0, 0, 0.63], [0, 0, 0, 1]]

        self.M_gripper = [[0, 0, 1, 0.474], [-1, 0, 0, 0], [0, -1, 0, 0.205], [0, 0, 0, 1]]


        self.M_hot_air = [[0, 0, 1, 0.374 + hot_air_len], [0, 1, 0, 0], [-1, 0, 0, 0.63], [0, 0, 0, 1]]
        self.Slist = np.array([[0, 0, 1, 0, 0, 0],
                          [0, 1, 0, -0.29, 0, 0],
                          [0, 1, 0, -0.56, 0, 0],
                          [1, 0, 0, 0, 0.63, 0],
                          [0, 1, 0, -0.63, 0, 0.302],
                          [1, 0, 0, 0, 0.63, 0],

                          [0, 0, -1, 0, 0.474, 0],
                          [0, 1, 0, -0.214, 0, 0.474],
                          [1, 0, 0, 0, 0.205, 0],
                          [1, 0, 0, 0, 0.205, 0]]).T
        self.joint_num = self.Slist.shape[1]
        self.joint_position = [0] * self.joint_num
        self.joint_position_received = False
        # arm_name should be l_arm or r_arm
        self.node = n
        self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
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

    def get_current_pos(self, frame='end'):
        while self.joint_position_received == False:
            rospy.loginfo('Waiting for joint angles...')
            rospy.sleep(1)
        return self.forward_kinematics(self.joint_position, frame)


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


    def move_to_joint_angle(self, angles, T, N):
        while self.joint_position_received == False:
            rospy.loginfo('Waiting for joint angles...')
            rospy.sleep(1)

        traj = mr.JointTrajectory(self.joint_position, angles, T, N, 5)
        dt = T/(N-1.0)
        self.move_joint_traj(traj, dt)

    def move_to_pose(self, pose, Tf, N, inter_type, frame='end'):
        while self.joint_position_received == False:
            rospy.loginfo('Waiting for joint angles...')
            rospy.sleep(1)

        # Cartisian or joint interplation
        if inter_type == 'c':
            x_start = self.forward_kinematics(self.joint_position, frame)
            print('xstart', x_start)
            x_end = pose
            print('xend', x_end)
            x_traj = mr.CartesianTrajectory(x_start, x_end, Tf, N, 5)
            #print(x_traj)
            traj = []
            for i in range(len(x_traj)):
                if i == 0:
                    theta0 = self.joint_position
                else:
                    theta0 = traj[i-1]
                #print(x_traj[i])
                #print(theta0)
                theta, _ = self.inverse_kinematics( x_traj[i], theta0, frame)
                #print(theta)
                traj.append(theta)
        elif inter_type == 'j':
            theta_d = self.inverse_kinematics(pose, self.joint_position, frame)
            traj = mr.JointTrajectory(self.joint_position, theta_d, Tf, N, 5)
        dt = Tf/(N-1.0)
        self.move_joint_traj(traj, dt)


    def joint_state_callback(self, data):
        for i in range(self.joint_num):
            self.joint_position[i] = data.position[i]
        self.joint_position_received = True

    def forward_kinematics(self, angles, frame='end'):
        if frame == 'camera':
            return mr.FKinSpace(self.M_camera, self.Slist, angles)
        elif frame == 'needle':
            return mr.FKinSpace(self.M_needle, self.Slist, angles)
        elif frame == 'gripper':
            return mr.FKinSpace(self.M_gripper, self.Slist, angles)
        elif frame == 'hot_air':
            return mr.FKinSpace(self.M_hot_air, self.Slist, angles)
        else:
            return mr.FKinSpace(self.M_end, self.Slist, angles)

    def inverse_kinematics(self, pose, thetalist0, frame='end'):
        if frame == 'camera':
            return mr.IKinSpace(self.Slist, self.M_camera, pose, thetalist0, 0.001, 0.0001)
        elif frame == 'needle':
            return mr.IKinSpace(self.Slist, self.M_needle, pose, thetalist0, 0.001, 0.0001)
        elif frame == 'gripper':
            return mr.IKinSpace(self.Slist, self.M_gripper, pose, thetalist0, 0.001, 0.0001)
        elif frame == 'hot_air':
            return mr.IKinSpace(self.Slist, self.M_hot_air, pose, thetalist0, 0.001, 0.0001)
        else:
            return mr.IKinSpace(self.Slist, self.M_end, pose, thetalist0, 0.001, 0.0001)



def main():
    n = rospy.init_node('joint_position_tester')
    arm = Arm('r_arm', n)
    arm.move_to_joint_angle([0]*arm.joint_num, 3, 100)
    print("move to home position!")
    rospy.sleep(3)
    angles = [0, 0, 0, 0, 0, 0, 0, 0, -0.5, 0.5]
    arm.move_to_joint_angle(angles, 3, 100)
    print("move to working home position!")
    rospy.sleep(3)
    print("current {} pose:".format('end'))
    T = arm.get_current_pos('gripper')
    print(T)
    print(Quaternion(matrix=T))
    '''
    p = [0]*6
    p[4] = pi/2
    arm.move_to_joint_angle(p, 1, 500)
    #arm.move_to_joint_angle([-0.3]*6, 3, 500)
    T = arm.get_current_pos()
    T[0, 3] += 0.1
    arm.move_to_pose(T, 2, 200, 'c')
    print("move to pose!")
    T = arm.get_current_pos()
    T[0, 3] -= 0.3
    arm.move_to_pose(T, 2, 200, 'c')
    '''
    rospy.spin()


if __name__ == '__main__':
    main()
