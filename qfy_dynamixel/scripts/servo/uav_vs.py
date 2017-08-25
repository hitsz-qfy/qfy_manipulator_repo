#!/usr/bin/env python
# coding=utf-8

import rospy
import math
from hybrid_vs_control import HybridVS
from qfy_dynamixel.msg import multi_joint_point
from qfy_dynamixel.srv import *
from control_msgs.msg import FollowJointTrajectoryActionResult
from dynamixel_msgs.msg import MotorStateFloatList
from std_msgs.msg import Bool

grasp_flag = False
init_position = False
trajectory_done = False

cur_joint = multi_joint_point()
cur_joint.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
cur_joint.data = [0.,0.,0.,0.,0.,0.,0.]

check_joint_flag = Bool()
land_flag = Bool()

def handle_init_position(req):
    global init_position
    rospy.loginfo("Initialize the position of manipulator at time: %f"%req.header.stamp.secs)
    init_position = True
    return True

def handle_grasp_call(req):
    global grasp_flag
    time = req.header.stamp
    rospy.loginfo("Begin to grasp at time : %f"%time.secs)
    grasp_flag = True
    return True

def ax_check_callback(msg):
    global cur_joint
    for motor_state in msg.motor_states:
        cur_joint.data[motor_state.id - 1] = motor_state.position


def mx_check_callback(msg):
    global cur_joint
    for motor_state in msg.motor_states:
        cur_joint.data[motor_state.id - 1] = motor_state.position

def check_joint_trj(joint_cur, joint_des):
    global trajectory_done, check_joint_flag
    sum = 0.
    if joint_cur[0] != 0.:
        for i in xrange(len(joint_cur)):
            # print("joint %d current value: %f"%(i, joint_cur[i]))
            # print("joint %d desired value: %f"%(i, joint_des[i]))
            sum += math.pow((joint_cur[i] - joint_des[i]),2)
        if sum < 0.1:
            trajectory_done = True
            check_joint_flag.data = True
            pub_check_joint_trj.publish(check_joint_flag)
        else:
            trajectory_done = False
            check_joint_flag = False
            pub_check_joint_trj.publish(check_joint_flag)


if __name__ == '__main__':
    rospy.init_node('hybrid_vs_control', anonymous=True)
    grasp_srv = rospy.Service('/begin_grasp', BeginGrasp, handle_grasp_call)
    init_position_srv = rospy.Service('/init_position', BeginGrasp, handle_init_position)

    sub_ax_state = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList,
                                    ax_check_callback)
    sub_mx_state = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList,
                                    mx_check_callback)
    pub_joint_goal_point = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)
    pub_check_joint_trj = rospy.Publisher('/check_joint_trj', Bool, queue_size=10)
    pub_land_signal = rospy.Publisher('/land_signal', Bool, queue_size=10)

    init_time = rospy.Time.now()
    hybrid_control = HybridVS()

    joint_p = multi_joint_point()
    joint_p.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
    joint_p.data = [0., 0.0, 0., 0.0, 0., 0.0, 0.0]

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        # rospy.loginfo(cur_joint.data)

        # check_joint_trj(cur_joint.data, joint_p.data)

        #######################################
        ########## initialization and ready to visual ########
        if init_position:
            joint_p.header.stamp = rospy.Time.now()
            joint_p.data = [1.57, 1.55, 1.2, 0.0, 0.5, 0.0, -0.8]
            pub_joint_goal_point.publish(joint_p)
            check_joint_trj(cur_joint.data, joint_p.data)
            if trajectory_done:
                rospy.loginfo("Initialization Done!!")
                init_position = False
                trajectory_done = False

        if grasp_flag:
            joint_p.header.stamp = now
            if now - init_time < rospy.Duration(5):
                joint_p.data = [1.57, 1.0, 0.5, 0.0, 0.4, 0.0, 0.0]
                pub_joint_goal_point.publish(joint_p)
                rospy.loginfo_throttle(60, "Trying to find Apriltags!!")
            else:
                # hybrid_control.pbvs.tracker.get_now_tf()

                ########################################
                ############ visual and grasp #############
                hybrid_control.pbvs.tracker.get_theta_u()

                if hybrid_control.pbvs.tracker.get_now_tf() and not hybrid_control.grasp_flag:
                    if not hybrid_control.grasp_flag:

                        hybrid_control.pub_joint_data()
                        hybrid_control.move_uav_to_target()


                if hybrid_control.grasp_flag:

                    if hybrid_control.grasp():
                        land_flag.data = True
                        pub_land_signal.publish(land_flag)
                    else:
                        land_flag.data = False

        rate.sleep()


# if __name__ == '__main__':
#     rospy.init_node('track_client',anonymous=True)
#     grasp_srv = rospy.Service('/begin_grasp', BeginGrasp, handle_grasp_call)
#     init_position_srv = rospy.Service('/init_position', BeginGrasp , handle_init_position)
#     sub_trjresult = rospy.Subscriber('/m_arm_controller/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, result_callback)
#     pub = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)
# 
#     origin = []
#     joint_p = multi_joint_point()
#     joint_p.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
#     rate = rospy.Rate(50)
#     tf_cnt = 0
# 
#     time0 = rospy.Time()
#     tracker = track.Track()
# 
#     while not rospy.is_shutdown():
#         if init_position:
#             joint_p.header.stamp = rospy.Time.now()
#             joint_p.data = [1.57, 1.55, 1.2, 0.0, 0.5, 0.0, -0.8]
#             pub.publish(joint_p)
#             if trajectory_done:
#                 rospy.loginfo("Initialization Done!!")
#                 init_position = False
#                 trajectory_done = False
#         # Version 1: move when hovering , but no visual servoing
#         # if grasp_flag:
#         #     origin.append(rospy.Time.now())
#         #     now = rospy.Time.now()
#         #     joint_p.header.stamp = now
#         #     if now - origin[0] < rospy.Duration(5):
#         #         joint_p.data = [1.57, 0.33, 0.49, 0.0, -0.22, 0.0, -0.8]
#         #         rospy.loginfo_throttle(60, "Push Forward!")
#         #     elif now - origin[0] < rospy.Duration(10):
#         #         joint_p.data = [1.57, 1.55, 1.2, 0.0, 0.5, 0.0, -0.8]
#         #         rospy.loginfo_throttle(60, "Pull Back!")
#         #     else:
#         #         rospy.loginfo("Grasp Done!!!")
#         #         grasp_flag = False
#         #
#         #     pub.publish(joint_p)
# 
#         #Version 2: visual servoing
#         if grasp_flag:
#             origin.append(rospy.Time.now())
#             now = rospy.Time.now()
#             joint_p.header.stamp = now
#             if now - origin[0] < rospy.Duration(5):
#                 joint_p.data = [1.57, 1.0, 0.5, 0.0, 0.4, 0.0, 0.0]
#                 pub.publish(joint_p)
#                 rospy.loginfo_throttle(60, "Trying to find Apriltags!!")
#             else:
#                 if tf_cnt <= 100:
#                     if tracker.listener.frameExists('/camera') and tracker.listener.frameExists('/target1'):
#                         t = tracker.listener.getLatestCommonTime('/camera', '/target1')
#                         if rospy.Time.now().to_sec() - t.to_sec() < 0.2:
#                             (tracker.trans, tracker.rot) = tracker.listener.lookupTransform('/camera', '/target1', t)
#                             tracker.kine_calcu()
#                             tracker.tf_pub()
#                             tracker.desire_trans()
#                             if tracker.invekine():
#                                 tf_cnt += 1
#                                 rospy.logwarn_throttle(1, "Now Get target1, counter: %d" % tf_cnt)
#                             else:
#                                 rospy.logerr_throttle(1, "Get target, but no inverse kinematic solution!!!")
#                         else:
#                             rospy.logerr_throttle(1, "Time difference is : %f" % (rospy.Time.now().to_sec() - t.to_sec()))
#                         if tf_cnt == 90:
#                             time0.from_sec(t.to_sec())
#                 else:
#                     if tracker.get_current_Tc_cstar(time0):
#                         tracker.kine_calcu()
#                         tracker.tf_pub()
#                         tracker.desire_trans()
#                         tracker.publish_goal()
#                     else:
#                         rospy.logerr_throttle(0.5, "No tf get!!")
#                         # tracker.publish_goal()
# 
#         rate.sleep()





