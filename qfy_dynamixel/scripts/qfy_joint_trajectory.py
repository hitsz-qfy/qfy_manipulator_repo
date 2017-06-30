#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

cur_joint2_pos = 0.
real_pre_joint2_speed = 0.

pre_time = rospy.Time.from_sec(0.)


step_point = 0.

desired_position = 1.0
desired_speed = 0.3



####velocity pid controller#######
####### u is the step position ##########


def cur_joint2_cb(msg):
    global pre_time, real_pre_joint2_speed, cur_joint2_pos, step_point
    Kp = 0.3
    in_time = rospy.Time.now()
    cur_joint2_pos = msg.current_pos

    # rospy.loginfo("current pos: %f"%msg.current_pos)

    # if pre_time != rospy.Time.from_sec(0.):
    #     real_joint2_acceleration = (msg.velocity - real_pre_joint2_speed) /(in_time-pre_time).to_sec()
    #     rospy.loginfo("current accel: %lf " % (real_joint2_acceleration))


    error = desired_speed - msg.velocity
    step = error * Kp
    if error > 0:
        step_point = step + msg.current_pos
        if step_point >= desired_position:
            step_point = desired_position
    else:
        step_point = msg.current_pos - step
        if step_point <= desired_position:
            step_point = desired_position


    pre_time = in_time

    real_pre_joint2_speed = msg.velocity
    rospy.loginfo("current velocity: %f, step point : %f" % (msg.velocity,step_point))

    # rospy.logwarn("callback joint2 value: %f"%msg.current_pos)

def pub_data():
    global step_point,desired_position
    rospy.init_node('qfy_joint_trajectory', anonymous=True)

    tra_pub = rospy.Publisher('/joint2_controller/command', Float64, queue_size=10)
    init_sub = rospy.Subscriber('/joint2_controller/state', JointState, cur_joint2_cb)

    rospy.sleep(rospy.Duration(0.1))

    rate = rospy.Rate(20)

    # desired_position = 1

    rospy.loginfo("current joint2 value: %f"%cur_joint2_pos)
    # cur_joint2_value = 1.0
    # const_list = list(cur_joint2_value + cur_joint2_value * i / 100. for i in xrange(50))

    ###########position points list#############
    const_list = list(cur_joint2_pos + (desired_position - cur_joint2_pos) * i / 100. for i in xrange(101))
    counter = 0
    while not rospy.is_shutdown():
        # rospy.set_param("/joint2_controller/joint_speed", 0.2)
        # rospy.loginfo(const_list[counter])
        # tra_pub.publish(const_list[counter])

        # rospy.loginfo(step_point)
        tra_pub.publish(step_point)

        # if counter != 99:
        #     counter += 1

        rate.sleep()


if __name__ == '__main__':
    try:
        pub_data()
    except rospy.ROSInterruptException:
        pass