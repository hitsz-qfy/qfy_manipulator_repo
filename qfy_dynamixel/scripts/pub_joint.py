#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @Author: qfyhaha
# @Date:   2016-11-17 16:33:06
# @Last Modified by:   qfyhaha
# @Last Modified time: 2017-01-10 11:00:23
# @Description: publish joint point to each joint

import rospy
from qfy_dynamixel.msg import multi_joint_point

def pub_data():
    pub = rospy.Publisher('joint_goal_point',multi_joint_point, queue_size=10)
    rospy.init_node('pub_jointpoint',anonymous=True)
    up_flag = False
    pub_mjp = multi_joint_point()
    pub_mjp.header.stamp = rospy.Time.now()
    pub_mjp.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

    joint2 = list(round((0.+2./100.*i),2) for i in xrange(50))

    init_joint = [1.57, 1.55, 1.2, 0.0, 0.0, 0.0, -0.8]
    pub_mjp.data = init_joint
    #pub_mjp.data = [1.3, -0.46, -0.66, 0.0, 0.34, 0.0, -0.01] #origin
    # pub_mjp.data = [1.57, 1.5, 1.2, 0.0, 0.4, 0.0, -0.6]
    # pub_mjp.data = [1.61, 0.33, 0.49, 0.0, -0.22, 0.0, 0.0]

    time_init = rospy.Time.now()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        pub_mjp.header.stamp = rospy.Time.now()
        pub.publish(pub_mjp)
        diff_time = now.to_sec() - time_init.to_sec()
        if diff_time > 5.0:
            if not up_flag:
                i = int(round(diff_time - int(diff_time),2) * 100)
                rospy.loginfo("up , i: %d"%i)
                pub_mjp.data[2] -= joint2[i / 2]
                if i == 98:
                    up_flag = True
            else:
                i = int(round(diff_time - int(diff_time), 2) * 100)
                rospy.loginfo("down, i: %d" % (100-i))
                pub_mjp.data[2] += joint2[i / 2]
                if i ==98:
                    up_flag = False

        rate.sleep()


if __name__ == '__main__':
    try:
        pub_data()
    except rospy.ROSInterruptException:
        pass
