#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @Author: qfyhaha
# @Date:   2016-11-17 16:33:06
# @Last Modified by:   qfyhaha
# @Last Modified time: 2017-01-10 11:00:23
# @Description: publish joint point to each joint

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_msgs.msg import MultiJointCommand
from dynamixel_msgs.msg import MotorStateList, MotorStateFloatList
from dynamixel_driver.dynamixel_const import *

cb_flag = False
joint4_position = 0.0
joint5_position = 0.0
joint6_position = 0.0
joint7_position = 0.0
cnt = 0

cur_joint4_position = 0.
cur_joint5_position = 0.
cur_joint6_position = 0.
cur_joint7_position = 0.

def check_feasible(data):
    if data[1] > 1.57:
        data[1] = 1.57
    if data[1] < -1.57:
        data[1] = -1.57
    if data[2] > 1.57:
        data[2] = 1.57
    if data[2] < -1.57:
        data[2] = -1.57

def ax_state_cb(msg):
    global cb_flag, joint4_position, joint5_position, joint6_position, joint7_position, cnt, \
        cur_joint4_position, cur_joint5_position, cur_joint6_position, cur_joint7_position
    cb_flag = True
    if cnt == 2:
        joint4_position = msg.motor_states[0].position
        joint5_position = msg.motor_states[1].position
        joint6_position = msg.motor_states[2].position
        joint7_position = msg.motor_states[3].position
    cnt += 1
    if msg.motor_states[1].speed != 0:
        rospy.logwarn("joint5 speed : %d, joint6 speed : %d"%(msg.motor_states[1].speed, msg.motor_states[2].speed))

    cur_joint4_position = msg.motor_states[0].position
    cur_joint5_position = msg.motor_states[1].position
    cur_joint6_position = msg.motor_states[2].position
    cur_joint7_position = msg.motor_states[3].position

    # rospy.loginfo_throttle(60, "joint mx 1: %f"%msg.actual.positions[0])
    # rospy.loginfo_throttle(1, "joint mx : %f ,  %f ,  %f" % (msg.motor_states[0].position, msg.motor_states[1].position, msg.motor_states[2].position))
    # rospy.loginfo("joint mx : %f ,  %f ,  %f" % (msg.motor_states[0].position, msg.motor_states[1].position, msg.motor_states[2].position))

def pub_data():
    global joint4_position, joint5_position, joint6_position, joint7_position, \
            cur_joint4_position, cur_joint5_position, cur_joint6_position, cur_joint7_position
    pub_joint_ax = rospy.Publisher('/ax_joint_controller/command', MultiJointCommand, queue_size=10)
    sub_joint_ax = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList, ax_state_cb)
    rospy.init_node('pub_joint_mx',anonymous=True)
    up_slope = True

    joint_ax_data = MultiJointCommand()
    joint_ax_data.joint_name = ['joint_4', 'joint_5', 'joint_6', 'joint_7']

    joint_list = list(round((0.+1./50.*i),2) for i in xrange(50))
    joint_speed_cmd = [0.4, 0.3, 0.5, 0.4]

    # time_init = rospy.Time.now()
    rate = rospy.Rate(40)
    i = 0

    if not cb_flag:
        rospy.sleep(rospy.Duration(0.01))
    if not len(joint_ax_data.joint_positions):
        joint_ax_data.joint_positions = [joint4_position, joint5_position, joint6_position, joint7_position]

    while not rospy.is_shutdown():
        time_now = rospy.Time.now()
        joint_ax_data.header.stamp = time_now

        if i < len(joint_list)-1 and up_slope:
            i += 1
            joint_speed_cmd = [0.4, 0.3, 0.5, 0.4]
            if i == len(joint_list)-1:
                up_slope = False
        else :
            i -= 1
            joint_speed_cmd = [0.4, 0.3, 0.5, 0.4]
            if i == 0 :
                up_slope = True

        joint_ax_data.joint_positions = [joint4_position, joint5_position + joint_list[i], joint6_position , joint7_position]
        # joint_ax_data.joint_positions = [1.57, 0.0, -1.0]

        joint_ax_data.joint_speed = joint_speed_cmd
        check_feasible(joint_ax_data.joint_positions)
        pub_joint_ax.publish(joint_ax_data)

        error_position = [joint_ax_data.joint_positions[0] - cur_joint4_position,
                          joint_ax_data.joint_positions[1] - cur_joint5_position,
                          joint_ax_data.joint_positions[2] - cur_joint6_position,
                          joint_ax_data.joint_positions[3] - cur_joint7_position]

        # rospy.logwarn("%s"%[joint4_position, joint5_position, joint7_position])

        rospy.loginfo_throttle(0.1, "joint_ax : %f , %f , %f, %f" % (joint_ax_data.joint_positions[0],
                                                                     joint_ax_data.joint_positions[1],
                                                                     joint_ax_data.joint_positions[2],
                                                                     joint_ax_data.joint_positions[3]))
        rospy.logwarn_throttle(0.1, "ax error : %s "%error_position)

        rate.sleep()

if __name__ == '__main__':
    try:
        pub_data()
    except rospy.ROSInterruptException:
        pass