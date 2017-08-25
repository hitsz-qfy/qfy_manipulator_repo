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

    joint_data.data = [1.57, 1.0, 0.5, 0.0, 0.4, 0.0, 0.0] ###origin
    # joint_data.data = [1.4, -0.49, -0.36, 0.0, 0.2, 0.0, 0.0]
    # joint_data.data = [1.52, 0.3, 0.0, 0.0, 0.4, 0.0, 0.0]
    # joint_data.data = [1.57, 0.5, 0.1, 0.0, 0.5, 0.0, 0.0]
    up_flag = True
    down_flag = False
    cnt = 0

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        joint_data.header.stamp = rospy.Time.now()
        # if up_flag:
        #     joint_data.data[1] += 0.01
        #     joint_data.data[2] += 0.01
        #     joint_data.data[4] += 0.01
        #     cnt += 1
        #     if cnt >50:
        #         up_flag = False
        #         down_flag = True
        #         cnt = 0
        #
        # if down_flag:
        #     joint_data.data[1] -= 0.01
        #     joint_data.data[2] -= 0.01
        #     joint_data.data[4] -= 0.01
        #     cnt += 1
        #     if cnt >50:
        #         up_flag = True
        #         down_flag = False
        #         cnt = 0
        # rospy.loginfo("joint 2: %.4f, joint 3: %.4f, joint 5: %.4f"%(joint_data.data[1], joint_data.data[2], joint_data.data[4]))
        pub.publish(joint_data)
        rate.sleep()