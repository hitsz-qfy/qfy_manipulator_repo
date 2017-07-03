#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from qfy_dynamixel.msg import multi_joint_point

if __name__ == '__main__':
    rospy.init_node('pbvs_control', anonymous=True)
    pub = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)

    joint_data = multi_joint_point()
    joint_data.header.stamp = rospy.Time.now()
    joint_data.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

    # joint_data.data = [1.57, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0] ###origin
    joint_data.data = [1.57, 1.0, 0.4, 0.0, 0.5, 0.0, 0.0]

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(joint_data)
        rate.sleep()