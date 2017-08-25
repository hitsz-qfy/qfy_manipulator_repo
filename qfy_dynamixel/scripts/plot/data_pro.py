#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
import tf
from math import pi
from dynamixel_msgs.msg import MotorStateFloatList
from qfy_dynamixel.msg import multi_joint_point
import message_filters


csvfile = file('joint_value.csv','wb')
writer = csv.writer(csvfile)

def result_callback(cur_joint, mx_state, ax_state):
    global csvfile, writer
    joint_value = []
    # rospy.loginfo("ok")
    rospy.logwarn("publish joint 2: %.4f, publish joint 3: %.4f, publish joint 5: %.4f"%(cur_joint.data[1], cur_joint.data[2], cur_joint.data[4]))
    rospy.loginfo("current joint 2: %.4f, current joint 3: %.4f, current joint 5: %.4f"%(mx_state.motor_states[1].position, mx_state.motor_states[2].position, ax_state.motor_states[1].position))
    # rospy.loginfo(msg[0])

    # pose = []
    # euler = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
    #                                           msg.pose.orientation.y,
    #                                           msg.pose.orientation.z,
    #                                           msg.pose.orientation.w])
    # pose.append(msg.pose.position.x)
    # pose.append(msg.pose.position.y)
    # pose.append(msg.pose.position.z)
    # pose.append(euler[0]*180/pi)
    # pose.append(euler[1]*180/pi)
    # pose.append(euler[2]*180/pi)

    # if not rospy.is_shutdown():
    #     try:
    #         writer.writerow(pose)
    #     except:
    #         csvfile.close()
    # else:
    #     csvfile.close()

def listener():
    rospy.init_node('listener_joint_value', anonymous=True)
    # rospy.sleep(20.0)
    rospy.loginfo("Start to record")

    sub_joint_pub = message_filters.Subscriber('/joint_goal_point', multi_joint_point)
    sub_joint_mx = message_filters.Subscriber('/mx_joint_controller/state', MotorStateFloatList)
    sub_joint_ax = message_filters.Subscriber('/ax_joint_controller/state', MotorStateFloatList)

    ts = message_filters.ApproximateTimeSynchronizer([sub_joint_pub, sub_joint_mx, sub_joint_ax], 40, 0.1, allow_headerless=True)

    ts.registerCallback(result_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()