#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @Author: qfyhaha
# @Date:   2016-11-17 16:33:06
# @Last Modified by:   qfyhaha
# @Last Modified time: 2017-01-10 11:00:23
# @Description: publish joint point to each joint

import rospy
from actionlib_msgs.msg import GoalStatusArray
from control_msgs.msg import FollowJointTrajectoryActionResult
from qfy_dynamixel.msg import multi_joint_point


def status_callback(msg):
    # if not hasattr(msg.status_list,'status'):
    #     print("Not Moving!!!!")
    # else:
    #     if msg.status_list.status == 1:
    #         print("Moving!!!")
    #     elif msg.status_list.status == 3:
    #         print("Stop!!!!")
    a = getattr(msg,'status_list')
    # rospy.loginfo(a)
    # print a[0].find('status')
    if len(a)<2:
        pass
    elif len(a) == 2 :
        print a[1]


# def result_callback(msg):
#     if msg.status.status == 3:
#         print("Trajectory End!!!")

def pub_data():
    pub = rospy.Publisher('joint_goal_point',multi_joint_point, queue_size=10)
    # sub_status = rospy.Subscriber('m_arm_controller/follow_joint_trajectory/status', GoalStatusArray, status_callback)
    # sub_result = rospy.Subscriber('m_arm_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, result_callback)
    rospy.init_node('pub_jointpoint',anonymous=True)

    pub_mjp = multi_joint_point()
    pub_mjp.header.stamp = rospy.Time.now()
    pub_mjp.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6','joint_7']

#    pub_mjp.data = [1.57, 2.2, 2.6, 0.0, 0.2, 0.0]
    pub_mjp.data = [1.57, 0.0, -1.3, 0.0, 0.0, 0.0, -0.6]

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        pub_mjp.header.stamp = rospy.Time.now()
        pub.publish(pub_mjp)
        rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    try:
        pub_data()
    except rospy.ROSInterruptException:
        pass
