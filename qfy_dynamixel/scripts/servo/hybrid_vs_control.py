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
        # self.tracker = track_2.Track()
        self.pbvs = pbvs_node.Pbvs()
        self.ibvs = ibvs_node.Ibvs()

        self.cur_mx_positions = [0., 0., 0.]
        self.cur_ax_positions = [0., 0., 0., 0.]
        self.lambd = 0.25
        self.linear_v = np.matrix((0., 0., 0.))
        self.angle_v = np.matrix((0., 0., 0.))
        self.joint_jacobian = np.asmatrix(np.zeros((6,6)))
        self.joint_vel = np.asmatrix(np.zeros((6,1)))

        self.filter_cnt = 0
        self.window_size = 50
        self.filter_sum = [0., 0., 0., 0., 0., 0., 0.]
        self.filter_out = [0., 0., 0., 0., 0., 0., 0.]

        self.pub_joint = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)
        self.sub_joint_mx = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList, self.mx_state_cb)
        self.sub_joint_ax = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList, self.ax_state_cb)

        # self.pub_error = rospy.Publisher('/pbvs_error', )

        self.joint_data = multi_joint_point()
        self.joint_data.data = [0., 0., 0., 0., 0., 0., 0.,]
        self.joint_data.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

    def mx_state_cb(self, msg):
        for motor_state in msg.motor_states:
            self.cur_mx_positions[(motor_state.id - msg.motor_states[0].id)] = motor_state.position

    def ax_state_cb(self, msg):
        for motor_state in msg.motor_states:
            self.cur_ax_positions[(motor_state.id - msg.motor_states[0].id)] = motor_state.position

    def get_cur_total_positions(self):
        self.cur_total_positions = self.cur_mx_positions + self.cur_ax_positions
        return self.cur_total_positions

    def hybrid_error(self):
        # rospy.loginfo(self.pbvs.calcu_error())
        # rospy.loginfo(self.ibvs.error_mat)
        return np.concatenate([self.pbvs.calcu_error(), self.ibvs.error_mat], axis=0)

    def hybrid_interaction_mat(self):
        # rospy.loginfo(self.pbvs.calcu_interaction_mat())
        # rospy.loginfo(self.ibvs.calcu_interaction_mat())
        return np.concatenate([self.pbvs.calcu_interaction_mat(), self.ibvs.calcu_interaction_mat()], axis=0)

    def get_camera_vel(self):
        interaction_weight_mat = ((self.ibvs.weight_mat * self.hybrid_interaction_mat()).T * (self.ibvs.weight_mat * self.hybrid_interaction_mat())).I * \
                                 (self.ibvs.weight_mat * self.hybrid_interaction_mat()).T
        camera_vel =  - self.lambd * interaction_weight_mat * self.ibvs.weight_mat * self.hybrid_error()
        self.linear_v = camera_vel[:3, 0]
        self.angle_v = camera_vel[3:6, 0]
        # rospy.loginfo("\nlinear velocity : %s, angle velocity : %s" % (self.linear_v, self.angle_v))

    def control_law(self):
        self.get_camera_vel()
        self.joint_jacobian = self.pbvs.tracker.get_joint_jacobian()
        # rospy.loginfo('\n'+'%s'%self.joint_jacobian)
        self.joint_vel = - np.dot(self.joint_jacobian.I, np.concatenate((self.linear_v, self.angle_v),axis=0))
        # rospy.loginfo('\n' + '%s' % self.joint_vel)

        return self.joint_vel

    def check_interaction_mat(self):
        return np.any(self.ibvs.calcu_interaction_mat()) and np.any(self.pbvs.calcu_interaction_mat())

    def check_get_tf(self):
        if self.pbvs.tracker.get_now_tf():
            return self.pbvs.tracker.get_tf
        else:
            return self.pbvs.tracker.get_tf

    def filter_data(self, list_):
        if self.filter_cnt < self.window_size:
            for i in xrange(7):
                self.filter_sum[i] += list_[i]
                # rospy.logwarn(i)
            self.filter_cnt += 1
        else:
            # rospy.logerr("error")
            self.filter_cnt = 0
            self.filter_out = list(self.filter_sum[i]/self.window_size for i in xrange(6)) + [0.]
            self.filter_sum = [0., 0., 0., 0., 0., 0., 0.]
            if self.filter_out[0] != 0.:
                self.joint_data.data = self.filter_out
                self.pub_joint.publish(self.joint_data)

    def pub_joint_data(self):
        # tmp = []
        # for i in xrange(6):
        #     if i < 3:
        #         self.control_law().itemset(i, (- self.control_law().item(i)))
        #     tmp += self.control_law().item(i)
        # self.joint_data.data = tmp
        self.get_cur_total_positions()
        if self.check_get_tf() and self.check_interaction_mat():
            for i in list(xrange(1,3)):
                self.joint_data.data[i] = self.cur_total_positions[i] - 2*self.control_law().item(i)
            self.joint_data.data[0] = self.cur_total_positions[0]
            self.joint_data.data[3] = 0.0
            self.joint_data.data[4] = self.cur_total_positions[4] + 0.2 * self.control_law().item(4)
            self.joint_data.data[5] = 0.
            # rospy.logwarn("\njoint velocity are : %s" % self.joint_data.data)
            # self.pub_joint.pub)lish(self.joint_data)
        else:
            # rospy.logerr("current joint")
            self.joint_data.data = self.get_cur_total_positions()

        self.filter_data(self.joint_data.data)
        # self.joint_data.data = self.filter_out
        # rospy.loginfo("joint positions : %s" % self.joint_data.data)
        # if self.joint_data.data[0] != 0.:
        #     # rospy.logwarn("Can pub")
        #     self.pub_joint.publish(self.joint_data)

if __name__ == '__main__':
    rospy.init_node('pbvs_control', anonymous=True)
    # pbvs_control = Pbvs()
    hybrid_control = HybridVS()

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        hybrid_control.pbvs.tracker.get_now_tf()
        hybrid_control.pbvs.tracker.get_theta_u()

        if hybrid_control.pbvs.tracker.rot:
            hybrid_control.pub_joint_data()
            # hybrid_control.hybrid_interaction_mat()

        rate.sleep()
