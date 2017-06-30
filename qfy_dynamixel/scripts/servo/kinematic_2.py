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
        # self.joint3_tmp = 0.

        self.sub_mx = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList, self.mx_joint_callback)
        self.sub_ax = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList, self.ax_joint_callback)

        ###----------deprecated------------------####
        # self.sub1 = rospy.Subscriber('/joint1_controller/state', JointState, self.joint1_callback)
        # self.sub2 = rospy.Subscriber('/joint2_controller/state', JointState, self.joint2_callback)
        # self.sub3 = rospy.Subscriber('/joint3_controller/state', JointState, self.joint3_callback)
        # self.sub4 = rospy.Subscriber('/joint4_controller/state', JointState, self.joint4_callback)
        # self.sub5 = rospy.Subscriber('/joint5_controller/state', JointState, self.joint5_callback)
        # self.sub6 = rospy.Subscriber('/joint6_controller/state', JointState, self.joint6_callback)
        # self.sub7 = rospy.Subscriber('/joint7_controller/state', JointState, self.joint7_callback)
        # self.pub = rospy.Publisher('joint_current_kinematic',TransformStamped,queue_size=10)
        ###----------deprecated------------------####

    def mx_joint_callback(self, msg):
        self.joint_value[0] = np.pi/2. - msg.motor_states[0].position
        self.joint_value[1] = - msg.motor_states[1].position
        self.joint_value[2] = np.pi/2. + msg.motor_states[2].position
        self.tmp_3 = msg.motor_states[2].position

    def ax_joint_callback(self, msg):
        self.joint_value[3] = msg.motor_states[0].position
        self.joint_value[4] = msg.motor_states[1].position
        self.joint_value[5] = msg.motor_states[2].position
        self.joint_value[6] = msg.motor_states[3].position

    # def joint1_callback(self,msg):
    #     # print("joint1_state: %s"%msg.current_pos)
    #     #true value (pi/2 ============== 0)
    #     # self.joint_value[0]=msg.current_pos - np.pi/2.
    #     self.joint_value[0] = np.pi/2. - msg.current_pos
    #
    # def joint2_callback(self,msg):
    #     # print("joint2_state: %s" % msg.current_pos)
    #     # self.joint_value[1]= 4.71 - msg.current_pos
    #     #true value (1.57-4.71 ============== -1.57-1.57)
    #     #self.joint_value[1] = msg.current_pos - np.pi #origin
    #     self.joint_value[1] = - msg.current_pos
    #
    # def joint3_callback(self,msg):
    #     # print("joint3_state: %s" % msg.current_pos)
    #     # self.joint_value[2]=msg.current_pos - 3.1415926
    #     #true value (1.57-4.71 ============== 3.14-0)
    #     # self.joint_value[2] = np.pi*1.5 - msg.current_pos #origin
    #     self.joint_value[2] = np.pi/2. + msg.current_pos
    #     self.tmp_3 = msg.current_pos
    #     # print("joint 3 value: %s"%msg.current_pos)
    #     # print("joint_value[2] value: %s"%self.joint_value[2])
    #
    # def joint4_callback(self,msg):
    #     # print("joint4_state: %s" % msg.current_pos)
    #     self.joint_value[3]=msg.current_pos
    #
    # def joint5_callback(self,msg):
    #     # print("joint5_state: %s" % msg.current_pos)
    #     # self.joint_value[4]=msg.current_pos + np.pi/2
    #     self.joint_value[4] = msg.current_pos
    #
    # def joint6_callback(self,msg):
    #     # print("joint6_state: %s" % msg.current_pos)
    #     self.joint_value[5]=msg.current_pos
    #
    # def joint7_callback(self,msg):
    #     # print("joint6_state: %s" % msg.current_pos)
    #     self.joint_value[6]=msg.current_pos

    def cur_joint(self):
    #     return [self.joint_value[0]+np.pi/2, -self.joint_value[1], self.tmp_3,  self.joint_value[3],
    #             self.joint_value[4], self.joint_value[5], self.joint_value[6]]
        return [np.pi/2.-self.joint_value[0], -self.joint_value[1], self.tmp_3,  self.joint_value[3],
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
        self.joint_value[2] += np.pi / 2
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

        self.T23 = np.mat([[cos(self.joint_value[2]), 0, sin(self.joint_value[2]), 0],
                           [sin(self.joint_value[2]), 0, -cos(self.joint_value[2]), 0],
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
        # self.joint_value[2] += np.pi/2
        # # print("kinematic : %s"%self.joint_value)
        #
        # self.T01 = np.mat([[cos(self.joint_value[0]), 0,  sin(self.joint_value[0]), -self.l2*cos(self.joint_value[0])],
        #               [sin(self.joint_value[0]), 0, -cos(self.joint_value[0]), -self.l2*sin(self.joint_value[0])],
        #               [0, 1, 0, 0],
        #               [0, 0, 0, 1]
        #               ])
        #
        # self.T12 = np.mat([[cos(self.joint_value[1]), -sin(self.joint_value[1]), 0,  self.l3*cos(self.joint_value[1])],
        #               [sin(self.joint_value[1]), cos(self.joint_value[1]),  0,  self.l3*sin(self.joint_value[1])],
        #               [0, 0, 1, 0],
        #               [0, 0, 0, 1]
        #               ])
        #
        # self.T23 = np.mat([[cos(self.joint_value[2]), 0, sin(self.joint_value[2]),  0],
        #               [sin(self.joint_value[2]), 0, -cos(self.joint_value[2]), 0],
        #               [0, 1, 0, 0],
        #               [0, 0, 0, 1]
        #               ])
        #
        # self.T34 = np.mat([[cos(self.joint_value[3]), 0, -sin(self.joint_value[3]), 0],
        #               [sin(self.joint_value[3]), 0, cos(self.joint_value[3]), 0],
        #               [0, -1, 0, self.l4],
        #               [0, 0, 0, 1]
        #               ])
        #
        # self.T45 = np.mat([[cos(self.joint_value[4]), 0, sin(self.joint_value[4]), 0],
        #               [sin(self.joint_value[4]), 0, -cos(self.joint_value[4]), 0],
        #               [0, 1, 0, 0],
        #               [0, 0, 0, 1]
        #               ])
        #
        # self.T56 = np.mat([[cos(self.joint_value[5]), -sin(self.joint_value[5]), 0, 0],
        #               [sin(self.joint_value[5]), cos(self.joint_value[5]), 0, 0],
        #               [0, 0, 1, self.l6],
        #               [0, 0, 0, 1]
        #               ])
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

            return self.jacobian_mat
