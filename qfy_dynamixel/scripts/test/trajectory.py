#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from qfy_dynamixel.msg import multi_joint_point
from kinematic_2 import Kinematic

# if __name__ == '__main__':
#     rospy.init_node('pbvs_control', anonymous=True)
#     pub = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)
#
#     joint_data = multi_joint_point()
#     joint_data.header.stamp = rospy.Time.now()
#     joint_data.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
#
#     # joint_data.data = [1.67, 1.0, 0.5, 0.0, 0.6, 0.0, -1.4] ###origin
#     joint_data.data = [1.67, -0.5, 0.0, 0., -0.4, 0., -1.4]
#
#     up_flag = True
#     down_flag = False
#     cnt = 0
#
#     rate = rospy.Rate(20)
#     while not rospy.is_shutdown():
#         joint_data.header.stamp = rospy.Time.now()
#         pub.publish(joint_data)
#         rate.sleep()


if __name__ == '__main__':
    rospy.init_node('trajectory', anonymous=True)
    kinematic = Kinematic(0.007,0.280,0.10387,0.095)

    pub = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)

    joint_data = multi_joint_point()
    joint_data.header.stamp = rospy.Time.now()
    joint_data.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
    joint_data.data = [0., 0., 0., 0., 0., 0., 0.]

    init_data = [1.67, 1.0, 0.5, 0.0, 0.6, 0.0, -1.4] ###origin

    desir_data = [1.67, -0.5, 0.0, 0., -0.4, 0., -1.4]

    diff_data =  [0., 0., 0., 0., 0., 0., 0.]
    for i in xrange(len(init_data)):
        diff_data[i] = desir_data[i] - init_data[i]

    cnt = 0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        joint_data.header.stamp = rospy.Time.now()


        j = kinematic.calcu_jacobian()
        u_, s_, d_ = np.linalg.svd(j)
        if s_[5] < 0.1:
            rospy.logwarn("Miniest singular value %f"%s_[5])

        if cnt <= 100:
            for i in xrange(len(init_data)):
                joint_data.data[i] = init_data[i] + diff_data[i] * cnt / 100
            cnt += 1
        rospy.loginfo(joint_data.data)
        pub.publish(joint_data)
        rate.sleep()