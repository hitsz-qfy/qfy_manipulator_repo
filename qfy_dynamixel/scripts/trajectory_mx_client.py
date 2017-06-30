#!/usr/bin/env python

# @Author:qfyhaha
# @Description:
import roslib

roslib.load_manifest('qfy_dynamixel')

import rospy
from trajectory_class import *


if __name__ == '__main__':
    rospy.init_node('trajectory_mx')
    arm_m = Joint('m_arm')
    rospy.spin()