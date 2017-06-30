#!/usr/bin/env python
# -*- coding: utf-8 -*

# @Author:qfyhaha
# @Description:

import rospy
from multi_joint_trj_class import *

if __name__ == '__main__':
    rospy.init_node('trajectory_mx')
    arm_m = Multijoint('mx')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        arm_m.move_joint()
        rate.sleep()
