#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @Author:qfyhaha
# @Description:

import rospy
from qfy_dynamixel.msg import multi_joint_point

from dynamixel_msgs.msg import MultiJointCommand, MotorStateFloatList
from dynamixel_msgs.msg import MotorStateFloatList
from dynamixel_driver.dynamixel_const import *

class Multijoint(object):
    def __init__(self, motor_name):
        # arm_name should be b_arm or f_arm
        self.name = motor_name
        self.joint_data_pre = []
        self.get_cmd = False
        self.get_vs = False

        self.multi_joint_data = MultiJointCommand()
        self.vs_joint_data = MultiJointCommand()
        self.joint_speed = [0.2,0.2,0.2,0.2,0.4,0.2,0.2]

        self.sub = rospy.Subscriber('/joint_goal_point', multi_joint_point, self.callback)
        # self.sub_vs = rospy.Subscriber('/joint_goal_point_vs', multi_joint_point, self.vs_callback)
        self.sub_vs = rospy.Subscriber('/vs_interplation', multi_joint_point, self.vs_callback)
        self.pub_joint = rospy.Publisher('/'+self.name[0]+'x_joint_controller/command', MultiJointCommand, queue_size=10)
        self.sub_joint = rospy.Subscriber('/'+self.name[0]+'x_joint_controller/state', MotorStateFloatList, self.state_callback)

        self.cur_joint = multi_joint_point()

        self.diff_joint = []
        self.procedure_cnt = 0
        self.total_interplation = 100

        self.init_joint = multi_joint_point()
        self.des_joint = multi_joint_point()

        # self.jta = actionlib.SimpleActionClient('/' + self.name + '_controller/follow_joint_trajectory',
        #                                         FollowJointTrajectoryAction)
        # rospy.loginfo('Waiting for joint trajectory action')
        # self.jta.wait_for_server()
        # rospy.loginfo('Found joint trajectory action!')

    def vs_callback(self,msg):
        self.vs_joint_data.header.stamp = rospy.Time.now()
        if self.name[0] == 'm':
            self.vs_joint_data.joint_name = msg.id[0:3]
            self.vs_joint_data.joint_positions = msg.data[0:3]
            self.vs_joint_data.joint_speed = self.joint_speed[0:3]

        else:
            self.vs_joint_data.joint_name = msg.id[3:7]
            self.vs_joint_data.joint_positions = msg.data[3:7]
            self.vs_joint_data.joint_speed = self.joint_speed[3:7]

        if self.joint_data_pre != self.vs_joint_data.joint_positions:
            rospy.logwarn('Found vs joint trajectory action!')
            self.get_vs = True
            self.get_cmd = False
            # self.move_joint(self.joint_data)

        self.joint_data_pre = self.vs_joint_data.joint_positions

    def state_callback(self,msg):
        if self.name[0] == 'm':
            self.cur_joint.id = ['joint_1', 'joint_2', 'joint_3']
            self.cur_joint.data = [0.,0.,0.]
            for motor_state in msg.motor_states:
                self.cur_joint.data[motor_state.id - 1] = motor_state.position
        else:
            self.cur_joint.id = ['joint_4', 'joint_5', 'joint_6','joint_7']
            self.cur_joint.data = [0.,0.,0.,0.]
            for motor_state in msg.motor_states:
                self.cur_joint.data[motor_state.id - 4] = motor_state.position

    def check_feasible(self, joint):
        if self.name[0] == 'm':
            for position in joint:
                if position > 1.57:
                    position = 1.57
                if position < -1.57:
                    position = -1.57

    def interpolation_joint(self):
        if len(self.init_joint.data) > 1 and len(self.des_joint.data) > 1:
            self.diff_joint = list(self.des_joint.data[i] - self.init_joint.data[i] for i in xrange(len(self.init_joint.data)))
            return self.diff_joint
        else:
            return False


    def move_joint(self):
        if self.get_cmd:
            self.check_feasible(self.des_joint.data)
            if self.interpolation_joint():
                if self.procedure_cnt < self.total_interplation:
                    self.multi_joint_data.joint_positions = list(self.init_joint.data[i] +
                                                      self.diff_joint[i] / self.total_interplation * self.procedure_cnt
                                                      for i in xrange(len(self.init_joint.data)))
                    self.procedure_cnt += 1
                    self.multi_joint_data.joint_speed[1] = 0.4
                    self.multi_joint_data.joint_speed[2] = 0.6

                self.pub_joint.publish(self.multi_joint_data)
            else:
                rospy.logerr_throttle(60, "Do not get enough joint data")

        if self.get_vs:
            self.check_feasible(self.vs_joint_data.joint_positions)
            self.pub_joint.publish(self.vs_joint_data)

        # goal = FollowJointTrajectoryGoal()
        # goal.trajectory.joint_names = self.joint_id
        # point = JointTrajectoryPoint()
        # point.positions = angles
        # point.time_from_start = rospy.Duration(2.0)
        # goal.trajectory.points.append(point)
        # self.jta.send_goal_and_wait(goal)
    def callback(self, msg):
        self.multi_joint_data.header.stamp = rospy.Time.now()
        if self.name[0] == 'm':
            self.multi_joint_data.joint_name = msg.id[0:3]
            self.multi_joint_data.joint_speed = self.joint_speed[0:3]

            self.des_joint.id = msg.id[0:3]
            self.des_joint.data = msg.data[0:3]

            self.init_joint.id = msg.id[0:3]
            self.init_joint.data = self.cur_joint.data
            # self.joint_id = msg.id[0:3]
            # self.joint_data = msg.data[0:3]
        else:
            self.multi_joint_data.joint_name = msg.id[3:7]
            self.multi_joint_data.joint_speed = self.joint_speed[3:7]

            self.des_joint.id = msg.id[3:7]
            self.des_joint.data = msg.data[3:7]

            self.init_joint.id = msg.id[3:7]
            self.init_joint.data = self.cur_joint.data

            # self.joint_id = msg.id[3:7]
            # self.joint_data = msg.data[3:7]

        if self.joint_data_pre != self.des_joint.data:
            rospy.loginfo('Found joint trajectory action!')
            self.get_cmd = True
            self.get_vs = False
            self.procedure_cnt = 0
            # self.move_joint(self.joint_data)

        self.joint_data_pre = self.des_joint.data
