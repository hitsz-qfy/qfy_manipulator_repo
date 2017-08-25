#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import Bool
from dynamixel_msgs.msg import MotorStateFloatList

ax_check = False
mx_check = False

def ax_check_callback(msg):
    for motor_state in msg.motor_states:
        if motor_state.moving:
            rospy.loginfo("id '%d' is moving"%motor_state.id)
        else:
            rospy.logwarn("id '%d' is not moving"%motor_state.id)

def mx_check_callback(msg):
    pass
    # for motor_state in msg.motor_states:
    #     print motor_state['id']


if __name__ == '__main__':
    rospy.init_node('check_joint_trj', anonymous=True)
    sub_ax_state = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList,
                                    ax_check_callback)
    sub_mx_state = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList,
                                    mx_check_callback)
    pub_check_result = rospy.Publisher('/check_joint_trj', Bool, queue_size=100)

    check_joint = Bool()
    check_joint.data = False

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        if ax_check and mx_check:
            check_joint.data = True
            rospy.loginfo("Joint stop!!!!")
            pub_check_result.publish(check_joint)
        else:
            check_joint.data = False

        rate.sleep()