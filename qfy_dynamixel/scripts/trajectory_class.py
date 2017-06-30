#!/usr/bin/env python

# @Author:qfyhaha
# @Description:
import roslib

roslib.load_manifest('qfy_dynamixel')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
from qfy_dynamixel.msg import multi_joint_point


class Joint:
    def __init__(self, motor_name):
        # arm_name should be b_arm or f_arm
        self.name = motor_name
        self.joint_id = []
        self.joint_data = []
        self.joint_data_pre = []

        self.sub = rospy.Subscriber('joint_goal_point',multi_joint_point,self.callback)

        self.jta = actionlib.SimpleActionClient('/' + self.name + '_controller/follow_joint_trajectory',
                                                FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        # rospy.loginfo('Found joint trajectory action!')

    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_id
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

    def callback(self,msg):
        if self.name[0] == 'm':
            self.joint_id = msg.id[0:3]
            self.joint_data = msg.data[0:3]
        else:
            self.joint_id = msg.id[3:7]
            self.joint_data = msg.data[3:7]
            self.joint_data = msg.data[3:7]

        if self.joint_data_pre != self.joint_data:
            rospy.loginfo('Found joint trajectory action!')
            self.move_joint(self.joint_data)
     
        self.joint_data_pre = self.joint_data


