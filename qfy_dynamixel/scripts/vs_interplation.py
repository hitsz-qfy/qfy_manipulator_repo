#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from qfy_dynamixel.msg import multi_joint_point
from dynamixel_msgs.msg import MotorStateFloatList
from std_msgs.msg import Float64

class VS_INTER(object):
    def __init__(self):
        self.vs_sub = rospy.Subscriber('/joint_goal_point_vs', multi_joint_point, self.vs_callback)
        self.mx_sub = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList, self.mx_callback)
        self.ax_sub = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList, self.ax_callback)
        self.keyboard_sub = rospy.Subscriber('/time_to_grasp', Float64, self.keyboard_callback)
        self.interplation_pub =  rospy.Publisher('/vs_interplation', multi_joint_point, queue_size=100)


        self.mx_cur_joint = multi_joint_point()
        self.mx_cur_joint.id = ['joint_1', 'joint_2', 'joint_3']
        self.mx_cur_joint.data = [0., 0., 0.]

        self.ax_cur_joint = multi_joint_point()
        self.ax_cur_joint.id = ['joint_4', 'joint_5', 'joint_6', 'joint_7']
        self.ax_cur_joint.data = [0., 0., 0., 0.]

        self.des_joint = multi_joint_point()
        self.des_joint.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        self.des_joint.data = [0., 0., 0., 0., 0., 0., 0.]

        self.cur_joint = multi_joint_point()
        self.cur_joint.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        self.cur_joint.data = [0., 0., 0., 0., 0., 0., 0.]

        self.pub_joint = multi_joint_point()
        self.pub_joint.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        self.pub_joint.data = [0., 0., 0., 0., 0., 0., 0.]

        self.diff_joint = []

        self.total_pro = 200
        self.pro_cnt = 0

        self.check_new_cnt = 0
        self.check_pre_cnt = 0

        self.grasp_flag = False
        self.emergency = False

    def keyboard_callback(self,msg):
        if msg.data == 1.0:
            self.grasp_flag = True
        if msg.data == 0.5:
            self.emergency = True

    def vs_callback(self, msg):
        self.des_joint.data = msg.data
        self.check_new_cnt += 1
        if self.check_new_cnt > 100:
            self.check_new_cnt = 0

    def mx_callback(self, msg):
        for motor_state in msg.motor_states:
            self.mx_cur_joint.data[motor_state.id - 1] = motor_state.position

    def ax_callback(self, msg):
        for motor_state in msg.motor_states:
            self.ax_cur_joint.data[motor_state.id - 4] = motor_state.position

    def check_updata(self):
        if self.check_new_cnt != self.check_pre_cnt:
            self.cur_joint.data = self.mx_cur_joint.data + self.ax_cur_joint.data

    def load_pre(self):
        self.check_pre_cnt = self.check_new_cnt

    def interplation(self):
        self.check_updata()
        cur_check = list(filter(lambda x : x != 0., self.cur_joint.data))
        des_check = list(filter(lambda x : x != 0., self.des_joint.data))
        if len(cur_check) != 0 and len(des_check) != 0:
            for i in xrange(len(self.cur_joint.data)):
                self.diff_joint = list(
                    self.des_joint.data[i] - self.cur_joint.data[i] for i in xrange(len(self.cur_joint.data)))

            self.total_pro = int(sorted(self.diff_joint, reverse=True)[0] / 0.01    )
            # rospy.logerr(self.total_pro)
            return True
        else:
            return False

    def publish_vs(self):
        if self.interplation() and not self.emergency and not self.grasp_flag:
            if self.pro_cnt < self.total_pro:
                self.pub_joint.data = list(self.cur_joint.data[i] +
                                                             self.diff_joint[i] / self.total_pro * self.pro_cnt
                                                             for i in xrange(len(self.cur_joint.data)))
                self.pro_cnt += 1
                # rospy.loginfo(self.pub_joint)
                self.interplation_pub.publish(self.pub_joint)
            else:
                self.pro_cnt = 0


if __name__ == '__main__':
    rospy.init_node('vs_interpaltion')
    vs_inter = VS_INTER()

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        vs_inter.publish_vs()
        vs_inter.load_pre()

        rate.sleep()