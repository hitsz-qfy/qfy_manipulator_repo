#!/usr/bin/env python
# coding=utf-8

import rospy
import math
from hybrid_vs_control import HybridVS
from qfy_dynamixel.msg import multi_joint_point
from qfy_dynamixel.srv import BeginGrasp
from control_msgs.msg import FollowJointTrajectoryActionResult
from dynamixel_msgs.msg import MotorStateFloatList
from std_msgs.msg import Bool
import tf

class GraspPrc(object):
    def __init__(self):
        self.grasp_flag = False
        self.init_position = False
        self.trajectory_done = False
        self.cur_joint = multi_joint_point()
        self.cur_joint.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        self.cur_joint.data = [0.,0.,0.,0.,0.,0.,0.]

        self.check_joint_flag = Bool()
        self.land_flag = Bool()

        self.init_grasp_time = rospy.Time.now()

        self.grasp_srv = rospy.Service('/begin_grasp', BeginGrasp, self.handle_grasp_call)
        self.init_position_srv = rospy.Service('/init_position', BeginGrasp, self.handle_init_position)

        self.sub_ax_state = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList,
                                        self.ax_check_callback)
        self.sub_mx_state = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList,
                                        self.mx_check_callback)
        self.pub_joint_goal_point = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)
        self.pub_check_joint_trj = rospy.Publisher('/check_joint_trj', Bool, queue_size=10)
        self.pub_land_signal = rospy.Publisher('/land_signal', Bool, queue_size=10)

    def handle_init_position(self,req):
        rospy.loginfo("Initialize the position of manipulator at time: %f" % req.header.stamp.secs)
        self.init_position = True
        return True

    def handle_grasp_call(self,req):
        rospy.loginfo("Begin to grasp at time : %f"%req.header.stamp.secs)
        self.grasp_flag = True
        self.init_grasp_time = rospy.Time.now()
        return True

    def ax_check_callback(self,msg):
        for motor_state in msg.motor_states:
            self.cur_joint.data[motor_state.id - 1] = motor_state.position

    def mx_check_callback(self,msg):
        for motor_state in msg.motor_states:
            self.cur_joint.data[motor_state.id - 1] = motor_state.position

    def check_joint_trj(self, joint_cur, joint_des):
        sum = 0.
        if joint_cur[0] != 0.:
            for i in xrange(len(joint_cur)):
                # print("joint %d current value: %f"%(i, joint_cur[i]))
                # print("joint %d desired value: %f"%(i, joint_des[i]))
                sum += math.pow((joint_cur[i] - joint_des[i]), 2)
            if sum < 0.1:
                self.trajectory_done = True
                self.check_joint_flag = True
                self.pub_check_joint_trj.publish(self.check_joint_flag)
            else:
                self.trajectory_done = False
                self.check_joint_flag = False
                self.pub_check_joint_trj.publish(self.check_joint_flag)


if __name__ == '__main__':
    rospy.init_node('hybrid_vs_control', anonymous=True)
    grasp_procedure = GraspPrc()
    hybrid_control = HybridVS()


    joint_p = multi_joint_point()
    joint_p.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
    joint_p.data = [0., 0., 0., 0., 0., 0., 0.]

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        # rospy.loginfo(cur_joint.data)

        # check_joint_trj(cur_joint.data, joint_p.data)

        #######################################
        ########## initialization and ready to visual ########
        if hybrid_control.emergency:
            hybrid_control.pub_joint_data()
            grasp_procedure.land_flag.data = True
            rospy.logwarn_throttle(60,"Emergency!!! Landing!!!!")
            if hybrid_control.check_arm_back():
                grasp_procedure.pub_land_signal.publish(grasp_procedure.land_flag)

        else:

            if grasp_procedure.init_position:
                joint_p.header.stamp = rospy.Time.now()
                joint_p.data = [1.57, 1.54, 1.2, 0.0, 0.5, 0.0, -0.8]#-0.8 close, 0.2 open
                grasp_procedure.pub_joint_goal_point.publish(joint_p)
                grasp_procedure.check_joint_trj(grasp_procedure.cur_joint.data, joint_p.data)

                if grasp_procedure.trajectory_done:
                    rospy.loginfo("Initialization Done!!")
                    grasp_procedure.init_position = False
                    grasp_procedure.trajectory_done = False
                    # init_time = rospy.Time.now()
                    # rospy.loginfo_throttle(60, "init_time : %f"%init_time.to_sec())

            if grasp_procedure.grasp_flag:
                joint_p.header.stamp = now

                if now - grasp_procedure.init_grasp_time < rospy.Duration(5):
                    joint_p.data = [1.57, 1.0, 0.5, 0.0, 0.4, 0.0, 0.2]
                    grasp_procedure.pub_joint_goal_point.publish(joint_p)
                    rospy.loginfo_throttle(60, "Trying to find Apriltags!!")
                else:
                    # hybrid_control.pbvs.tracker.get_now_tf()

                    ########################################
                    ############ visual and grasp #############
                    rospy.loginfo_throttle(60, "Hybrid visual procedure!!!")
                    # hybrid_control.pbvs.tracker.listener.waitForTransform("/camera", "/target1", rospy.Time.now(), rospy.Duration(4.0))
                    # try:
                    #
                    #     t = hybrid_control.pbvs.tracker.listener.getLatestCommonTime('/camera', '/target1')
                    #     # print t
                    #     # if self.listener.frameExists('target1'):
                    #     #     t = self.listener.getLatestCommonTime('/camera', '/target1')
                    #     if rospy.Time.now().to_sec() - t.to_sec() < 0.1:
                    #         (hybrid_control.pbvs.tracker.trans, hybrid_control.pbvs.tracker.rot) = \
                    #             hybrid_control.pbvs.tracker.listener.lookupTransform('/camera', '/target1', t)
                    #         # rospy.loginfo(tf.transformations.euler_from_quaternion(self.rot))
                    #         # rospy.logwarn(tf.transformations.quaternion_matrix(self.rot)[np.ix_([0,1,2],[0,1,2])])
                    #         # rospy.loginfo("\ntranslation: %s, \norientation: %s"%(self.trans, tf.transformations.euler_from_quaternion(self.rot)))
                    #         # rospy.loginfo(self.trans)
                    #         hybrid_control.pbvs.tracker.get_tf = True
                    #         rospy.logwarn_throttle(2, "Now Get target1")
                    #     else:
                    #         hybrid_control.pbvs.tracker.get_tf = False
                    #         rospy.logerr_throttle(1, "Time difference is : %f" % (rospy.Time.now().to_sec() - t.to_sec()))
                    #
                    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
                    #     rospy.logerr_throttle(1, "No target1")
                    #     continue

                    hybrid_control.pbvs.tracker.get_now_tf()

                    # hybrid_control.pbvs.tracker.get_theta_u()

                    if hybrid_control.pbvs.tracker.get_now_tf() and not hybrid_control.grasp_flag:
                        if not hybrid_control.grasp_flag:

                            hybrid_control.pub_joint_data()
                            # hybrid_control.get_fly_target_depth()
                            # hybrid_control.move_uav_to_target()

                    if hybrid_control.grasp_flag:
                        rospy.loginfo_throttle(60, "Begin to grasp!!!!")
                        if hybrid_control.grasp():
                            grasp_procedure.land_flag.data = True
                            rospy.loginfo_throttle(60,"landing flag is true")
                            grasp_procedure.pub_land_signal.publish(grasp_procedure.land_flag)
                        else:
                            grasp_procedure.land_flag.data = False

        rate.sleep()
