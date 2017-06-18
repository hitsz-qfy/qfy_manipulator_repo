#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @Author: qfyhaha
# @Date:   2016-11-17 16:33:06
# @Last Modified by:   qfyhaha
# @Last Modified time: 2017-01-10 11:00:23
# @Description: publish joint point to each joint

import rospy
from qfy_dynamixel.msg import multi_joint_point
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

def joint2_cb(msg):
    pass

def pub_data():
    # pub_joint2 = rospy.Publisher('/joint2_controller/command', Float64, queue_size=10)
    pub_joint3 = rospy.Publisher('/joint3_controller/command', Float64, queue_size=10)
    sub_joint2 = rospy.Subscriber('/joint2_controller/state', JointState, joint2_cb)
    rospy.init_node('pub_joint3',anonymous=True)
    up_slope = True

    joint3_data = Float64()
    joint_list = list(round((0.+1./100.*i),2) for i in xrange(100))

    time_init = rospy.Time.now()
    rate = rospy.Rate(40)
    i = 0
    while not rospy.is_shutdown():
        time_now = rospy.Time.now()

        if i < len(joint_list)-1 and up_slope:
            i += 1
            if i == len(joint_list)-1:
                up_slope = False
        else :
            i -= 1
            if i == 0 :
                up_slope = True

        joint3_data.data = joint_list[i]
        pub_joint3.publish(joint3_data)
        rospy.loginfo_throttle(0.1, "joint3 : %f"%(joint3_data.data))
        rate.sleep()


if __name__ == '__main__':
    try:
        pub_data()
    except rospy.ROSInterruptException:
        pass
