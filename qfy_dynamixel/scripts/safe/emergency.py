#!/usr/bin/env python
# coding=utf-8

import rospy
from mavros_msgs.msg import RCIn
from qfy_dynamixel.msg import multi_joint_point

flag = False

def result_callback(msg):
    global flag
    if len(msg.channels) != 0:
        if msg.channels[6]<1500:
            rospy.loginfo("NO CHANGE!!")
        else:
            rospy.loginfo("Safe Mode!!")
            pub_handle.data = [1.57, 1.5, 1.5, 0, 0, 0, -0.8]
            flag = True
            # pub_handle.data = [1.57, 1.65, 1.72,  0, 0, 0, 0.45]

if __name__ == '__main__':
    global flag
    rospy.init_node('safety_mode',anonymous=True)
    rc_sub = rospy.Subscriber('mavros/rc/in', RCIn, result_callback)
    pub = rospy.Publisher('joint_goal_point', multi_joint_point, queue_size=10)

    rate = rospy.Rate(20)
    pub_handle = multi_joint_point()
    pub_handle.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

    while not rospy.is_shutdown():
        if flag == True:
            pub.publish(pub_handle)
        rate.sleep()