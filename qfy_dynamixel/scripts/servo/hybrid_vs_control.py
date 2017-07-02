#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import track_2
from qfy_dynamixel.srv import *
import pbvs_node
import ibvs_node

from dynamixel_driver.dynamixel_const import *
from dynamixel_msgs.msg import MotorStateFloatList
from qfy_dynamixel.msg import multi_joint_point, PbvsErrorStamped

class HybridVS(object):
    def __init__(self):
        self.tracker = track_2.Track()
        self.pbvs = pbvs_node.Pbvs()
        self.ibvs = ibvs_node.Ibvs()

        self.cur_mx_positions = [0., 0., 0.]
        self.cur_ax_positions = [0., 0., 0., 0.]
        self.lambd = 0.025
        self.linear_v = np.matrix((0., 0., 0.))
        self.angle_v = np.matrix((0., 0., 0.))
        self.joint_jacobian = np.asmatrix(np.zeros((6,6)))
        self.joint_vel = np.asmatrix(np.zeros((6,1)))

        self.filter_cnt = 0
        self.window_size = 100
        self.filter_sum = [0., 0., 0., 0., 0., 0., 0.]
        self.filter_out = [0., 0., 0., 0., 0., 0., 0.]

        self.pub_joint = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)
        self.sub_joint_mx = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList, self.mx_state_cb)
        self.sub_joint_ax = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList, self.ax_state_cb)

        # self.pub_error = rospy.Publisher('/pbvs_error', )

        self.joint_data = multi_joint_point()
        self.joint_data.data = [0., 0., 0., 0., 0., 0., 0.,]
        self.joint_data.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']


    def hybrid_error(self):
        pass

    def hybrid_interaction_mat(self):
        pass

    def mx_state_cb(self, msg):
        for motor_state in msg.motor_states:
            self.cur_mx_positions[(motor_state.id - msg.motor_states[0].id)] = motor_state.position

    def ax_state_cb(self, msg):
        for motor_state in msg.motor_states:
            self.cur_ax_positions[(motor_state.id - msg.motor_states[0].id)] = motor_state.position

    def get_cur_total_positions(self):
        self.cur_total_positions = self.cur_mx_positions + self.cur_ax_positions
        return self.cur_total_positions

    def get_camera_vel(self):
        self.linear_v = - self.lambd * (self.tracker.get_tcstar_o_mat() - self.tracker.get_tc_o_mat() + np.cross(self.tracker.get_tc_o_mat(), self.tracker.get_theta_u()))
        self.angle_v = - self.lambd * self.tracker.get_theta_u()
        # rospy.loginfo("\nlinear velocity : %s, angle velocity : %s" % (self.linear_v, self.angle_v))

    def get_error(self):
        self.error_translation = - self.tracker.get_tcstar_o_mat() + self.tracker.get_tc_o_mat()
        self.error_orientation = self.tracker.get_theta_u()
        # rospy.loginfo("translation error: %s , rotation error: %s"%(self.error_translation, self.error_orientation))

    def control_law(self):
        self.get_camera_vel()
        self.joint_jacobian = self.tracker.get_joint_jacobian()
        # rospy.loginfo('\n'+'%s'%self.joint_jacobian)
        self.joint_vel = np.dot(self.joint_jacobian.I, np.row_stack((self.linear_v.T, self.angle_v.T)))
        # rospy.loginfo('\n' + '%s' % self.joint_vel)

        return self.joint_vel

    def check_get_tf(self):
        if self.tracker.get_now_tf():
            return self.tracker.get_tf
        else:
            return self.tracker.get_tf

    def filter_data(self, list_):
        if self.filter_cnt < self.window_size:
            for i in xrange(7):
                self.filter_sum[i] = list_[i]
            self.filter_cnt += 1
        else:
            self.filter_cnt = 0
            self.filter_out = list(self.filter_sum[i]/self.window_size for i in xrange(6))
            self.filter_sum = [0., 0., 0., 0., 0., 0., 0.]

    def pub_joint_data(self):
        # tmp = []
        # for i in xrange(6):
        #     if i < 3:
        #         self.control_law().itemset(i, (- self.control_law().item(i)))
        #     tmp += self.control_law().item(i)
        # self.joint_data.data = tmp
        self.get_cur_total_positions()
        if self.check_get_tf():
            for i in list(xrange(1,3)):
                self.joint_data.data[i] = self.cur_total_positions[i] + self.control_law().item(i)
            self.joint_data.data[0] = self.cur_total_positions[0]
            self.joint_data.data[3] = 0.0
            self.joint_data.data[4] = self.cur_total_positions[4] + self.control_law().item(4)
            self.joint_data.data[5] = 0.
            # rospy.logwarn("\njoint velocity are : %s" % self.joint_data.data)
            # self.pub_joint.publish(self.joint_data)
        else:
            self.joint_data.data = self.get_cur_total_positions()

        self.filter_data(self.joint_data.data)
        self.joint_data.data = self.filter_out
        rospy.loginfo("joint positions : %s" % self.joint_data.data)
        self.pub_joint.publish(self.joint_data)

if __name__ == '__main__':
    rospy.init_node('pbvs_control', anonymous=True)
    pbvs_control = Pbvs()
    # pub_joint = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)
    # sub_joint_mx = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList, mx_state_cb)
    # sub_joint_ax = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList, ax_state_cb)

    # tracker = track_2.Track()
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        # tracker.get_now_tf()
        # rospy.loginfo(tracker.rot)
        # tracker.get_theta_u()
        pbvs_control.tracker.get_now_tf()
        pbvs_control.tracker.get_theta_u()
        # rospy.loginfo(tracker.rot)
        if pbvs_control.tracker.rot:

            ## camera velocity####
            # pbvs_control.get_camera_vel()
            # linear_v = - lambd * (tracker.get_tcstar_o_mat() - tracker.get_tc_o_mat() + np.cross(tracker.get_tc_o_mat(), tracker.get_theta_u()))
            # angle_v = - lambd * tracker.get_theta_u()

            ## control law #####
            # pbvs_control.control_law()
            # joint_jacobian = tracker.get_joint_jacobian()
            # joint_vel = np.dot(joint_jacobian.I, np.row_stack((linear_v.T, angle_v.T)))

            # rospy.loginfo("\nlinear velocity : %s, angle velocity : %s" % (linear_v, angle_v))
            # rospy.logwarn("\njoint velocity are : %s" % joint_vel)
            # pbvs_control.get_error()
            # pbvs_control.pub_joint_data()
            pbvs_control.pub_error()
        rate.sleep()
