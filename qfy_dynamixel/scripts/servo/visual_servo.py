#!/usr/bin/env python
# coding=utf-8


import rospy
from hybrid_vs_control import HybridVS


if __name__ == '__main__':
    rospy.init_node('pbvs_control', anonymous=True)
    # pbvs_control = Pbvs()
    hybrid_control = HybridVS()

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        hybrid_control.pbvs.tracker.get_now_tf()
        # hybrid_control.pbvs.tracker.get_theta_u()

        if hybrid_control.pbvs.tracker.get_now_tf() and not hybrid_control.grasp_flag:
            if not hybrid_control.grasp_flag:

                hybrid_control.pub_joint_data()
                hybrid_control.pbvs.pub_error()

                # if hybrid_control.get_fly_target_depth() > 0.55:
                #     hybrid_control.move_uav_to_target()

            # hybrid_control.hybrid_interaction_mat()
        if hybrid_control.grasp_flag:
                # rospy.loginfo("haha")
                hybrid_control.grasp()

        rate.sleep()