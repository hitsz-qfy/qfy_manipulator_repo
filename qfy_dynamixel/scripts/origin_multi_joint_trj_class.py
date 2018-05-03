#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @Author:qfyhaha
# @Description:

import rospy
from qfy_dynamixel.msg import multi_joint_point

from dynamixel_msgs.msg import MultiJointCommand
from dynamixel_msgs.msg import MotorStateFloatList
from dynamixel_driver.dynamixel_const import *

class Multijoint(object):
    def __init__(self, motor_name):
        # arm_name should be b_arm or f_arm
        self.name = motor_name
        self.joint_data_pre = []
        self.get_cmd = False

        self.multi_joint_data = MultiJointCommand()
        self.joint_speed = [0.2,0.2,0.2,0.2,0.4,0.2,0.2]

        self.sub = rospy.Subscriber('/joint_goal_point', multi_joint_point, self.callback)
        self.pub_joint = rospy.Publisher('/'+self.name[0]+'x_joint_controller/command', MultiJointCommand, queue_size=10)

        # self.jta = actionlib.SimpleActionClient('/' + self.name + '_controller/follow_joint_trajectory',
        #                                         FollowJointTrajectoryAction)
        # rospy.loginfo('Waiting for joint trajectory action')
        # self.jta.wait_for_server()
        # rospy.loginfo('Found joint trajectory action!')

    def check_feasible(self, joint):
        if self.name[0] == 'm':
            for position in joint:
                if position > 1.57:
                    position = 1.57
                if position < -1.57:
                    position = -1.57

    def move_joint(self):
        if self.get_cmd:
            self.check_feasible(self.multi_joint_data.joint_positions)
            self.pub_joint.publish(self.multi_joint_data)

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
            self.multi_joint_data.joint_positions = msg.data[0:3]
            self.multi_joint_data.joint_speed = self.joint_speed[0:3]
            # self.joint_id = msg.id[0:3]
            # self.joint_data = msg.data[0:3]
        else:
            self.multi_joint_data.joint_name = msg.id[3:7]
            self.multi_joint_data.joint_positions = msg.data[3:7]
            self.multi_joint_data.joint_speed = self.joint_speed[3:7]
            # self.joint_id = msg.id[3:7]
            # self.joint_data = msg.data[3:7]

        if self.joint_data_pre != self.multi_joint_data.joint_positions:
            rospy.loginfo('Found joint trajectory action!')
            # self.move_joint(self.joint_data)

        self.joint_data_pre = self.multi_joint_data.joint_positions
        self.get_cmd = True


