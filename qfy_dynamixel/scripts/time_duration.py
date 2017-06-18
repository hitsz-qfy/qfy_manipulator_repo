#!/usr/bin/env python
# coding=utf-8
# Description: after 5s, publish different value
# Tips: sleep can block the program

import rospy
from std_msgs.msg import Float64
from qfy_dynamixel.msg import multi_joint_point
from std_srvs.srv import Trigger

flag_move = False

def trigger_service(req):
    global flag_move
    # req.success = True
    # req.message = 'Trigger Success'
    rospy.loginfo("service call")
    flag_move = True
    return True

if __name__ == '__main__':
    try:
        rospy.init_node('pub_jointpoint', anonymous=True)
        origin = rospy.Time.now()
        pub = rospy.Publisher('joint_goal_point', multi_joint_point, queue_size=10)
        service = rospy.Service('move_manipulator_trigger',Trigger, trigger_service)

        joint_p = multi_joint_point()
        joint_p.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6','joint_7']

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if flag_move:
                now = rospy.Time.now()
                joint_p.header.stamp = now
                if now-origin < rospy.Duration(5):
                    joint_p.data = [1.57, 0.5, 0.2, 0.0, 0.0, 0.0, -0.8]
                    rospy.loginfo_throttle(1.0, "Push Forward!")
                elif now-origin < rospy.Duration(10):
                    joint_p.data = [1.57, 1.5, 1.5, 0.0, 0.0, 0.0, -0.8]
                    rospy.loginfo_throttle(1.0, "Pull Back!")

                pub.publish(joint_p)
            # print (now-origin).to_sec(), a
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
