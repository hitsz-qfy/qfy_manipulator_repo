#!/usr/bin/env python
# coding=utf-8

import rospy
from  math import *
import numpy as np
from qfy_dynamixel.msg import multi_joint_point
from dynamixel_msgs.msg import JointState , MotorStateFloatList, MotorStateFloat

from geometry_msgs.msg import TransformStamped
import tf

class Kinematic(object):
    def __init__(self,l2,l3,l4,l6):
        self.joint_id=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7']
        # self.joint_value=[0.,0.,0.,0.,0.,0.,np.pi/2.]
        self.joint_value = [0., 0., 0., 0., 0., 0., 0.]

        self.kinematic= np.mat(np.zeros((4,4)))
        self.jacobian_mat = np.mat(np.zeros((6,6)))
        self.jacobian_array = np.asarray(self.jacobian_mat)
        self.l2, self.l3 , self.l4, self.l6= l2, l3, l4, l6
        self.tmp_3 = 0.
        # self.joint3_tmp = 0.

        self.sub_mx = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList, self.mx_joint_callback)
        self.sub_ax = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList, self.ax_joint_callback)



    def mx_joint_callback(self, msg):
        self.joint_value[0] = 1.57 - msg.motor_states[0].position
        self.joint_value[1] = - msg.motor_states[1].position
        self.joint_value[2] = np.pi/2. + msg.motor_states[2].position
        self.tmp_3 = msg.motor_states[2].position + np.pi
        # rospy.loginfo(self.tmp_3)

    def ax_joint_callback(self, msg):
        self.joint_value[3] = msg.motor_states[0].position
        self.joint_value[4] = msg.motor_states[1].position
        # if -0.2 < self.joint_value[4] < 0.2:
        #     rospy.loginfo('joint 5 current value: %f'%self.joint_value[4])
        self.joint_value[5] = msg.motor_states[2].position
        self.joint_value[6] = msg.motor_states[3].position


    def cur_joint(self):
    #     return [self.joint_value[0]+np.pi/2, -self.joint_value[1], self.tmp_3,  self.joint_value[3],
    #             self.joint_value[4], self.joint_value[5], self.joint_value[6]]
        return [1.57 - self.joint_value[0], -self.joint_value[1], self.joint_value[2] - np.pi/2,  self.joint_value[3],
                self.joint_value[4], self.joint_value[5], self.joint_value[6]]

    def kinematic_(self, joint_value):
        joint_value[2] += np.pi/2

        T01_ = np.mat([[cos(joint_value[0]), 0, sin(joint_value[0]), -self.l2 * cos(joint_value[0])],
                      [sin(joint_value[0]), 0, -cos(joint_value[0]), -self.l2 * sin(joint_value[0])],
                      [0, 1, 0, 0],
                      [0, 0, 0, 1]
                      ])

        T12_ = np.mat([[cos(joint_value[1]), -sin(joint_value[1]), 0, self.l3 * cos(joint_value[1])],
                      [sin(joint_value[1]), cos(joint_value[1]), 0, self.l3 * sin(joint_value[1])],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]
                      ])

        T23_ = np.mat([[cos(joint_value[2]), 0, sin(joint_value[2]), 0],
                      [sin(joint_value[2]), 0, -cos(joint_value[2]), 0],
                      [0, 1, 0, 0],
                      [0, 0, 0, 1]
                      ])

        T34_ = np.mat([[cos(joint_value[3]), 0, -sin(joint_value[3]), 0],
                      [sin(joint_value[3]), 0, cos(joint_value[3]), 0],
                      [0, -1, 0, self.l4],
                      [0, 0, 0, 1]
                      ])

        T45_ = np.mat([[cos(joint_value[4]), 0, sin(joint_value[4]), 0],
                      [sin(joint_value[4]), 0, -cos(joint_value[4]), 0],
                      [0, 1, 0, 0],
                      [0, 0, 0, 1]
                      ])

        T56_ = np.mat([[cos(joint_value[5]), -sin(joint_value[5]), 0, 0],
                      [sin(joint_value[5]), cos(joint_value[5]), 0, 0],
                      [0, 0, 1, self.l6],
                      [0, 0, 0, 1]
                      ])
        return T01_*T12_*T23_*T34_*T45_*T56_

    def derive_kine(self):
        # self.joint_value[2] += np.pi / 2
        # print("kinematic : %s"%self.joint_value)

        self.T01 = np.mat([[cos(self.joint_value[0]), 0, sin(self.joint_value[0]), -self.l2 * cos(self.joint_value[0])],
                           [sin(self.joint_value[0]), 0, -cos(self.joint_value[0]),
                            -self.l2 * sin(self.joint_value[0])],
                           [0, 1, 0, 0],
                           [0, 0, 0, 1]
                           ])

        self.T12 = np.mat([[cos(self.joint_value[1]), -sin(self.joint_value[1]), 0, self.l3 * cos(self.joint_value[1])],
                           [sin(self.joint_value[1]), cos(self.joint_value[1]), 0, self.l3 * sin(self.joint_value[1])],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]
                           ])

        self.T23 = np.mat([[cos(self.tmp_3), 0, sin(self.tmp_3), 0],
                           [sin(self.tmp_3), 0, -cos(self.tmp_3), 0],
                           [0, 1, 0, 0],
                           [0, 0, 0, 1]
                           ])

        self.T34 = np.mat([[cos(self.joint_value[3]), 0, -sin(self.joint_value[3]), 0],
                           [sin(self.joint_value[3]), 0, cos(self.joint_value[3]), 0],
                           [0, -1, 0, self.l4],
                           [0, 0, 0, 1]
                           ])

        self.T45 = np.mat([[cos(self.joint_value[4]), 0, sin(self.joint_value[4]), 0],
                           [sin(self.joint_value[4]), 0, -cos(self.joint_value[4]), 0],
                           [0, 1, 0, 0],
                           [0, 0, 0, 1]
                           ])

        self.T56 = np.mat([[cos(self.joint_value[5]), -sin(self.joint_value[5]), 0, 0],
                           [sin(self.joint_value[5]), cos(self.joint_value[5]), 0, 0],
                           [0, 0, 1, self.l6],
                           [0, 0, 0, 1]
                           ])

    def kinematic_calcu(self):
        self.derive_kine()
        self.kinematic = self.T01 * self.T12 * self.T23 * self.T34 * self.T45 * self.T56
        # print self.kinematic
        return self.kinematic

    def calcu_jacobian(self):
        # self.kinematic_calcu()
        if self.kinematic_calcu().size:
            self.T02 = self.T01 * self.T12
            self.T03 = self.T02 * self.T23
            self.T04 = self.T03 * self.T34
            self.T05 = self.T04 * self.T45
            self.T06 = self.T05 * self.T56
            # rospy.loginfo(self.T06)
            # rospy.logwarn(self.joint_value[2]) ######## joint_value[2] has problem ##########


            z_0 = np.matrix((0,0,1)).T
            z_1 = self.T01[:3, 2]
            z_2 = self.T02[:3, 2]
            z_3 = self.T03[:3, 2]
            z_4 = self.T04[:3, 2]
            z_5 = self.T05[:3, 2]

            p_0 = np.matrix((0,0,0)).T
            p_1 = self.T01[:3, 3]
            p_2 = self.T02[:3, 3]
            p_3 = self.T03[:3, 3]
            p_4 = self.T04[:3, 3]
            p_5 = self.T05[:3, 3]
            p_6 = self.T06[:3, 3]

            j_p1 = np.cross(z_0.T, (p_6 - p_0).T)
            j_p2 = np.cross(z_1.T, (p_6 - p_1).T)
            j_p3 = np.cross(z_2.T, (p_6 - p_2).T)
            j_p4 = np.cross(z_3.T, (p_6 - p_3).T)
            j_p5 = np.cross(z_4.T, (p_6 - p_4).T)
            j_p6 = np.cross(z_5.T, (p_6 - p_5).T)

            self.jacobian_mat = np.row_stack((np.column_stack((j_p1.T, j_p2.T, j_p3.T, j_p4.T, j_p5.T, j_p6.T)),
                                              np.column_stack((z_0, z_1, z_2, z_3, z_4, z_5))))

            self.jacobian_array = np.asarray(self.jacobian_mat)
            # rospy.loginfo(self.jacobian_mat)
            return self.jacobian_mat
