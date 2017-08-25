#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import track_2
from qfy_dynamixel.srv import *
import pbvs_node
import ibvs_node
import tf
import math
from numpy.linalg import inv

from dynamixel_driver.dynamixel_const import *
from dynamixel_msgs.msg import MotorStateFloatList
from qfy_dynamixel.msg import multi_joint_point, PbvsErrorStamped
from std_msgs.msg import Float64
import matplotlib.pyplot as plt

from qfy_dynamixel.srv import MoveUAV
from geometry_msgs.msg import PoseStamped


class HybridVS(object):
    def __init__(self):
        # self.tracker = track_2.Track()
        self.pbvs = pbvs_node.Pbvs()
        self.ibvs = ibvs_node.Ibvs()

        self.cur_mx_positions = [0., 0., 0.]
        self.cur_ax_positions = [0., 0., 0., 0.]
        self.lambd = 0.2
        self.dls_const = 0.01
        self.linear_v = np.matrix((0., 0., 0.))
        self.angle_v = np.matrix((0., 0., 0.))
        self.joint_jacobian = np.asmatrix(np.zeros((6,6)))
        self.joint_vel = np.asmatrix(np.zeros((6,1)))
        self.end_vel = np.asmatrix(np.zeros((6,1)))

        self.filter_cnt = 0
        self.window_size = 2
        self.filter_sum = [0., 0., 0., 0., 0., 0., 0.]
        self.filter_out = [0., 0., 0., 0., 0., 0., 0.]

        # self.height_const = rospy.get_param('/qfy_hover_node/beginpoint/z')
        self.pub_joint = rospy.Publisher('/joint_goal_point', multi_joint_point, queue_size=10)

        self.pub_uav_move = rospy.Publisher('/uav_move', PoseStamped, queue_size=10)
        self.pub_mavros_vision = rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.mavros_vision_cb)

        self.sub_keyboard = rospy.Subscriber('/time_to_grasp', Float64, self.keyboard_cb)
        self.sub_joint_mx = rospy.Subscriber('/mx_joint_controller/state', MotorStateFloatList, self.mx_state_cb)
        self.sub_joint_ax = rospy.Subscriber('/ax_joint_controller/state', MotorStateFloatList, self.ax_state_cb)

        self.pub_joint1_vel = rospy.Publisher('/joint1_vel', Float64, queue_size=10)
        self.pub_joint2_vel = rospy.Publisher('/joint2_vel', Float64, queue_size=10)
        self.pub_joint3_vel = rospy.Publisher('/joint3_vel', Float64, queue_size=10)
        self.pub_joint4_vel = rospy.Publisher('/joint4_vel', Float64, queue_size=10)
        self.pub_joint5_vel = rospy.Publisher('/joint5_vel', Float64, queue_size=10)
        self.pub_joint6_vel = rospy.Publisher('/joint6_vel', Float64, queue_size=10)

        self.pub_camera_x_trans = rospy.Publisher('/camera_linear_x', Float64, queue_size=10)
        self.pub_camera_y_trans = rospy.Publisher('/camera_linear_y', Float64, queue_size=10)
        self.pub_camera_z_trans = rospy.Publisher('/camera_linear_z', Float64, queue_size=10)

        self.pub_camera_x_angle = rospy.Publisher('/camera_angle_x', Float64, queue_size=10)
        self.pub_camera_y_angle = rospy.Publisher('/camera_angle_y', Float64, queue_size=10)
        self.pub_camera_z_angle = rospy.Publisher('/camera_angle_z', Float64, queue_size=10)


        # self.pub_error = rospy.Publisher('/pbvs_error', )
        self.uav_set_position = PoseStamped()
        self.uav_vision_pose = PoseStamped()

        self.joint_data = multi_joint_point()
        self.joint_data.data = [0., 0., 0., 0., 0., 0., 0.,]
        self.joint_data.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

        self.grasp_joint_data = multi_joint_point()
        self.grasp_joint_data.data = [0., 0., 0., 0., 0., 0., 0., ]
        self.grasp_joint_data.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

        self.joint1_vel = Float64()
        self.joint2_vel = Float64()
        self.joint3_vel = Float64()
        self.joint4_vel = Float64()
        self.joint5_vel = Float64()
        self.joint6_vel = Float64()

        self.plt_time = []
        self.plt_value = []

        self.pre_joint1 = 0.
        self.pre_joint6 = 0.
        self.lowpass_co = 0.1
        self.pre_camera_vel = []
        self.flag = False
        self.grasp_flag = False
        self.first_grasp = True
        self.arrival = False
        self.first_grasp_time = rospy.Time()
        self.time_now = rospy.Time()
        self.fly_target_depth = 0.


        self.const_vel_matrix = np.matrix([[0.,-1.,0., 0.05,0.,0.],
                                          [1.,0.,0.,   0.,0.05, -0.05],
                                          [0.,0.,1.,   0.05, 0.,0.],
                                          [0.,0.,0., 0.,-1.,0.],
                                          [0.,0.,0., 1.,0.,0.],
                                          [0.,0.,0., 0., 0.,1.]])

        self.srv_move_uav = MoveUAV()

    def mavros_vision_cb(self,msg):
        self.uav_vision_pose.pose.position.x = msg.pose.position.x
        self.uav_vision_pose.pose.position.y = msg.pose.position.y
        self.uav_vision_pose.pose.position.z = msg.pose.position.z
        self.uav_vision_pose.pose.orientation.w = msg.pose.orientation.w
        self.uav_vision_pose.pose.orientation.x = msg.pose.orientation.x
        self.uav_vision_pose.pose.orientation.y = msg.pose.orientation.y
        self.uav_vision_pose.pose.orientation.z = msg.pose.orientation.z

    def keyboard_cb(self,msg):
        if msg.data == 1.0:
            self.grasp_flag = True
        if msg.data == 0.:
            self.grasp_flag = False

    def mx_state_cb(self, msg):
        for motor_state in msg.motor_states:
            self.cur_mx_positions[(motor_state.id - msg.motor_states[0].id)] = motor_state.position

    def ax_state_cb(self, msg):
        for motor_state in msg.motor_states:
            self.cur_ax_positions[(motor_state.id - msg.motor_states[0].id)] = motor_state.position

    def get_cur_total_positions(self):
        self.cur_total_positions = self.cur_mx_positions + self.cur_ax_positions
        return self.cur_total_positions

    def hybrid_error(self):
        # rospy.loginfo("pbvs error: %s"%self.pbvs.calcu_error())
        # rospy.loginfo("ibvs error: %s"%self.ibvs.error_mat)
        return np.concatenate([self.pbvs.calcu_error(), self.ibvs.error_mat], axis=0)

    def hybrid_interaction_mat(self):
        # rospy.loginfo(self.pbvs.calcu_interaction_mat())
        # rospy.loginfo(self.ibvs.calcu_interaction_mat())
        return np.concatenate([self.pbvs.calcu_interaction_mat(), self.ibvs.calcu_interaction_mat()], axis=0)

    #######problem : 不能仅仅考虑旋转量的转换，还有平移量###############
    def get_end_effector_vel(self, camera_vel):
        self.end_vel = self.const_vel_matrix * camera_vel   #######problem#########
        # rospy.logwarn(self.end_vel)


        self.linear_v.itemset(0, - camera_vel.item(1))
        self.linear_v.itemset(1, camera_vel.item(0))
        self.linear_v.itemset(2, camera_vel.item(2))

        self.angle_v.itemset(0, - camera_vel.item(4))
        self.angle_v.itemset(1, camera_vel.item(3))
        self.angle_v.itemset(2, camera_vel.item(5))

        # self.linear_v.itemset(0, camera_vel.item(0))
        # self.linear_v.itemset(1, camera_vel.item(1))
        # self.linear_v.itemset(2, camera_vel.item(2))
        #
        # self.angle_v.itemset(0, camera_vel.item(3))
        # self.angle_v.itemset(1, camera_vel.item(4))
        # self.angle_v.itemset(2, camera_vel.item(5))
        # rospy.logwarn("\nlinear vel: %s, angle vel: %s"%(self.linear_v, self.angle_v))

        self.pub_camera_x_trans.publish(self.linear_v.item(0))
        self.pub_camera_y_trans.publish(self.linear_v.item(1))
        self.pub_camera_z_trans.publish(self.linear_v.item(2))

        self.pub_camera_x_angle.publish(self.angle_v.item(0))
        self.pub_camera_y_angle.publish(self.angle_v.item(1))
        self.pub_camera_z_angle.publish(self.angle_v.item(2))

    ########### OK ######################
    ############# need max value filter camera velocity ################
    def get_camera_vel(self):
        ############## IBVS control ##################
        ##### pseudo-inverse #######
        # self.flag = False
        # ibvs_int_mat = self.ibvs.calcu_interaction_mat()
        # camera_vel_ibvs = - 0.4 * inv(ibvs_int_mat.T * ibvs_int_mat) * ibvs_int_mat.T * self.ibvs.error_mat
        # camera_vel_ibvs.itemset()

        # # print len(self.pre_camera_vel)
        # camera_vel_tmp = camera_vel.copy()
        # camera_vel_tmp =  camera_vel_tmp.T
        # camera_list =  camera_vel_tmp.tolist()[0]
        # if len(self.pre_camera_vel):
        #     bias_list = [camera_list[i] - self.pre_camera_vel[i] for i in xrange(len(self.pre_camera_vel))]
        #     # bias_list = [camera_vel.T.tolist()[0][i] - self.pre_camera_vel[i] for i in xrange(len(self.pre_camera_vel))]
        #     if len(filter(lambda x: x != 0, bias_list)):
        #         tmp = [math.fabs(bias_list[i] / camera_list[i]) for i in xrange(len(self.pre_camera_vel))]
        #         for i in xrange(len(self.pre_camera_vel)):
        #             if tmp[i] > 0.3:
        #                 # rospy.logerr("OVER")
        #                 self.flag = True
        #                 break
        #         if not self.flag:
        #             self.get_end_effector_vel(camera_vel)
        #             rospy.loginfo("camera vel : %s" % (camera_vel))
        #             # print camera_vel
        #         else:
        #             self.get_end_effector_vel(np.asmatrix(self.pre_camera_vel).T)
        #             rospy.loginfo("camera vel : %s" % (np.asmatrix(self.pre_camera_vel).T))
        #
        # self.pre_camera_vel = camera_list


        ###### transpose ##########
        # camera_vel = - 0.2 * ibvs_int_mat.T * self.ibvs.error_mat
        # rospy.loginfo_throttle(0.1, camera_vel)

        ############## PBVS control ##################
        # rospy.logwarn(self.pbvs.calcu_error())
        # # rospy.loginfo(self.pbvs.calcu_interaction_mat())
        # camera_vel_pbvs = - self.lambd * self.pbvs.calcu_interaction_mat() * self.pbvs.calcu_error()
        # rospy.loginfo("camera vel : %s" % (camera_vel))
        # self.get_end_effector_vel(camera_vel)

        ############## Hybrid control ###################
        hybrid_int_mat = self.hybrid_interaction_mat()
        hybrid_err = self.hybrid_error()
        interaction_weight_mat = inv((self.ibvs.weight_mat * hybrid_int_mat).T * (self.ibvs.weight_mat * hybrid_int_mat)) * \
                                 (self.ibvs.weight_mat * hybrid_int_mat).T
        camera_vel_hybridvs =  - self.lambd * interaction_weight_mat * self.ibvs.weight_mat * hybrid_err

        # rospy.loginfo_throttle(0.1, "camera vel : %s" % (camera_vel))
        # camera_vel.itemset(0, - camera_vel.item(0))
        # camera_vel.itemset(1, - camera_vel.item(1))
        # camera_vel.itemset(2, - camera_vel.item(2))
        # rospy.loginfo_throttle(0.1, "camera vel : %s" % (camera_vel_pbvs))
        # self.get_end_effector_vel(camera_vel_ibvs)
        # self.get_end_effector_vel(camera_vel_pbvs)
        self.get_end_effector_vel(camera_vel_hybridvs)


    ######## PROBLEM!!!!!!! ##############
    ########## OUT PUT HUGE OSCILLATION ABOUT ZERO###########
    def control_law(self):

        self.get_camera_vel()
        # self.pub_camera_x_trans.publish(self.linear_v[0])
        # self.pub_camera_y_trans.publish(self.linear_v[1])
        # self.pub_camera_z_trans.publish(self.linear_v[2])
        #
        # self.pub_camera_x_angle.publish(self.angle_v[0])
        # self.pub_camera_y_angle.publish(self.angle_v[1])
        # self.pub_camera_z_angle.publish(self.angle_v[2])
        self.joint_jacobian = self.pbvs.tracker.get_joint_jacobian() ########### singularity PROBLEM ###########
        # rospy.loginfo('\n'+'%s'%self.joint_jacobian)
        # rospy.logwarn("\nlinear velocity: \n%s , \nangular velocity: \n%s"%(self.linear_v, self.angle_v))
        # rospy.logerr(self.joint_jacobian.I)
        # rospy.logwarn(self.pbvs.tracker.kine.joint_value[4])
        ######original#######
        # self.joint_vel = - np.dot(self.joint_jacobian.I, np.concatenate((self.linear_v, self.angle_v),axis=0))
        #####################

        ########DLS(Damped Least-Square)#######
        tmp = self.joint_jacobian * self.joint_jacobian.T + self.dls_const * \
                                                            self.dls_const * np.asmatrix(np.identity(6))
        # rospy.logwarn(tmp)

        tmp_coff = self.joint_jacobian.T * inv(tmp)

        # rospy.loginfo(tmp_coff)

        self.joint_vel = - np.dot(tmp_coff, np.concatenate((self.linear_v.T, self.angle_v.T), axis=0))
        # self.joint_vel = - np.dot(tmp_coff, self.end_vel)
        ######################################

        # rospy.loginfo('\n' + '%s' % self.joint_vel)
        # rospy.logwarn("joint 4: %.4f"%self.joint_vel.item(3))
        # rospy.loginfo("\njoint vel: %s"%self.joint_vel)             ##########problem: huge oscillation #########3
        self.joint1_vel = self.joint_vel.item(0)
        self.joint2_vel = self.joint_vel.item(1)
        self.joint3_vel = self.joint_vel.item(2)
        self.joint4_vel = self.joint_vel.item(3)
        self.joint5_vel = self.joint_vel.item(4)
        self.joint6_vel = self.joint_vel.item(5)
        # rospy.loginfo("\njoint1_vel: %.5f\njoint2_vel: %.5f\njoint3_vel: %.5f\njoint4_vel: %.5f\njoint5_vel: %.5f\njoint6_vel: %.5f"%
        #               (self.joint_vel.item(0), self.joint_vel.item(1), self.joint_vel.item(2), self.joint_vel.item(3), self.joint_vel.item(4), self.joint_vel.item(5)))
        if -0.05< (self.joint4_vel + self.joint6_vel) < 0.05:
            self.joint_vel.itemset(3, 0.)
            self.joint_vel.itemset(5, 0.)
        else:
            self.joint_vel.itemset(3, (self.joint4_vel + self.joint6_vel))
            self.joint_vel.itemset(5, 0.)

        self.pub_joint1_vel.publish(self.joint1_vel)
        self.pub_joint2_vel.publish(self.joint2_vel)
        self.pub_joint3_vel.publish(self.joint3_vel)
        self.pub_joint4_vel.publish(self.joint4_vel)
        self.pub_joint5_vel.publish(self.joint5_vel)
        self.pub_joint6_vel.publish(self.joint6_vel)

        return self.joint_vel

    def check_interaction_mat(self):
        return np.any(self.ibvs.calcu_interaction_mat()) and np.any(self.pbvs.calcu_interaction_mat())

    def check_get_tf(self):
        if self.pbvs.tracker.get_now_tf():
            return self.pbvs.tracker.get_tf
        else:
            return self.pbvs.tracker.get_tf

    def filter_data(self, list_):
        list_[0] = self.lowpass_co * list_[0] + (1 - self.lowpass_co) * self.pre_joint1
        list_[5] = self.lowpass_co * list_[5] + (1 - self.lowpass_co) * self.pre_joint6
        if self.filter_cnt < self.window_size:
            for i in xrange(7):
                self.filter_sum[i] += list_[i]
                # rospy.logwarn(i)
            self.filter_cnt += 1
        else:
            # rospy.logerr("error")
            self.filter_cnt = 0
            self.filter_out = list(self.filter_sum[i]/self.window_size for i in xrange(6)) + [0.]
            self.filter_sum = [0., 0., 0., 0., 0., 0., 0.]
            if self.filter_out[0] != 0.:
                # rospy.logwarn("filte();r output: %s"%self.filter_out)
                self.joint_data.data = self.filter_out
                self.pub_joint.publish(self.joint_data)
        self.pre_joint1 = list_[0]
        self.pre_joint6 = list_[5]
    ####deprecated#####
    # def check_control_law(self):
    #     if self.control_law().item(1) < 0 and self.control_law().item(2) < 0:
    #         return True
    #     else:
    #         return False
    ##################

    def check_region(self):
        if (np.square(self.joint_vel.item(1)) + np.square(self.joint_vel.item(2))) > 0.06:
            return True
        else:
            return False

    def pub_joint_data(self):
        # tmp = []
        # for i in xrange(6):
        #     if i < 3:
        #         self.control_law().itemset(i, (- self.control_law().item(i)))
        #     tmp += self.control_law().item(i)
        # self.joint_data.data = tmp
        # self.get_cur_total_positions()
        if self.check_get_tf() and self.check_interaction_mat():
            # for i in list(xrange(1,3)):
            #     self.joint_data.data[i] = self.cur_total_positions[i] - 1.4 * self.control_law().item(i)
            # rospy.logwarn("Get interaction")
            self.control_law()

            # if self.joint_vel.item(1) > 0.01 or self.joint_vel.item(1) < -0.01 and (self.pbvs.tracker.get_tc_o_mat().item(2) > 0.22 or self.pbvs.tracker.get_tc_o_mat().item(2) < 0.20 ):
            if (self.joint_vel.item(1) > 0.01 or self.joint_vel.item(1) < -0.01) \
                    and (self.joint_vel.item(0) > 0.03 or self.joint_vel.item(0) < -0.03):
                # rospy.loginfo("joint 2 :%s, joint 3: %s"%(self.control_law().item(1), self.control_law().item(2)))
                # rospy.logwarn("joint 2 : %.4f"%self.joint_vel.item(1))
                # rospy.logwarn("joint 3 input: %.4f"%self.joint_vel.item(2))
                # rospy.logerr("joint 5 : %.4f"%self.joint_vel.item(4))
                self.get_cur_total_positions()
                self.joint_data.data[0] = self.cur_total_positions[0] - 0.6 * self.joint_vel.item(0)
                # self.joint_data.data[0] = self.cur_total_positions[0] + 1.2 * self.control_law().item(0)
                self.joint_data.data[1] = self.cur_total_positions[1] + 0.6 * self.joint_vel.item(1)
                self.joint_data.data[2] = self.cur_total_positions[2] + 0.6 * self.joint_vel.item(2)

                # self.joint_data.data[3] = 0.0
                self.joint_data.data[3] = self.cur_total_positions[3]
                self.joint_data.data[4] = self.cur_total_positions[4] - 0.3 * self.joint_vel.item(4)
                # self.joint_data.data[5] = 0.
                self.joint_data.data[5] = self.cur_total_positions[5]

                tmp = list(self.joint_data.data[i] - self.cur_total_positions[i] for i in xrange(6))

                # rospy.loginfo("\njoint vel: %s"%tmp)
                self.filter_data(self.joint_data.data)

        else:
            # rospy.logerr("current joint")
            self.joint_data.data = self.get_cur_total_positions()
            # self.pub_joint.publish(self.joint_data)

        # self.filter_data(self.joint_data.data)

    def check_arrival(self):
        cur_joint = self.pbvs.tracker.get_cur_joint()
        goal_joint = self.grasp_joint_data.data
        rospy.loginfo("\ncurrent joint : %s\ngoal joint : %s"%(cur_joint, goal_joint))
        err = [0., 0., 0., 0., 0., 0., 0.]
        sum_err_square = 0.
        # rospy.logerr("cur_joint : %d"%len(cur_joint))
        # for i in xrange(len(cur_joint)):
        #     err[i] = cur_joint[i] - goal_joint[i]
        #     sum_err_square += err[i]* err[i]
        # rospy.logerr("sum_err_square: %f"%sum_err_square)
        # if sum_err_square < 0.07:
        #     self.arrival = True

        if math.fabs(cur_joint[2] - goal_joint[2]) < 0.02 and math.fabs(cur_joint[1] - goal_joint[1]) < 0.02:
            self.arrival = True
            return True
        else:
            return False

    def grasp(self):
        self.time_now = rospy.Time.now()
        # self.check_arrival()
        if self.pbvs.tracker.get_now_tf():
            self.pbvs.tracker.kine_calcu()
            # rospy.loginfo(self.pbvs.tracker.kine_calcu())
            self.pbvs.tracker.tf_pub()
            self.pbvs.tracker.desire_trans()

            if self.first_grasp:
                if self.pbvs.tracker.invekine():
                    self.grasp_joint_data.data = self.pbvs.tracker.invekine()
                    # rospy.logwarn(self.joint_data.data)
                    self.grasp_joint_data.data[3] = 0.
                    self.grasp_joint_data.data[5] = 0.
                    self.first_grasp_time = rospy.Time.now()
                    rospy.loginfo("Time to grasp at %f "%self.first_grasp_time.to_sec())
                    self.pub_joint.publish(self.grasp_joint_data)
                    self.first_grasp = False
                else:
                    rospy.logerr("No invekine!!")
        if self.grasp_joint_data.data[0] != 0.:
            self.check_arrival()
        if self.arrival:
            rospy.logwarn_throttle(60, "Arrive!!!")
            self.grasp_joint_data.data[6] = -0.9
            self.pub_joint.publish(self.grasp_joint_data)
            if math.fabs(self.pbvs.tracker.get_cur_joint()[6] + 0.9) < 0.1:
                self.grasp_joint_data.data = [1.57, 1.5, 1.1, 0.0, 0.5, 0.0, -0.9]
                self.pub_joint.publish(self.grasp_joint_data)
                if math.fabs(self.pbvs.tracker.get_cur_joint()[1] - 1.5) < 0.1 and \
                    math.fabs(self.pbvs.tracker.get_cur_joint()[2] - 1.1) < 0.1 and \
                    math.fabs(self.pbvs.tracker.get_cur_joint()[4] - 0.5) < 0.1 and \
                    math.fabs(self.pbvs.tracker.get_cur_joint()[6] + 0.9) < 0.1 :
                    return True
        return False


        ##############需要完成从摄像头得到逆解，然后算出期望的关节，在规划运动########
        #############还需要完成距离加权##############

    def get_fly_target_depth(self):
        self.pbvs.tracker.kine_calcu()
        self.pbvs.tracker.tf_pub()
        self.pbvs.tracker.desire_trans()
        self.fly_target_depth = self.pbvs.tracker.Tb_grasp_mat.item(2,3) + 0.15
        rospy.logwarn("target depth to uav: %f"%hybrid_control.fly_target_depth)
        return self.fly_target_depth

    def move_uav_to_target(self):
        err_x = self.fly_target_depth - 0.55
        err_y = self.pbvs.tracker.Tb_grasp_mat.item(1,3)
        err_yaw = math.atan2(err_y, err_x)
        norm_err_x = err_x / math.sqrt(math.pow(err_x,2) + math.pow(err_y,2))
        norm_err_y = err_y / math.sqrt(math.pow(err_x,2) + math.pow(err_y,2))

        self.uav_set_position.header.stamp = rospy.Time.now()
        self.uav_set_position.pose.position.x = err_x + self.uav_vision_pose.pose.position.x
        self.uav_set_position.pose.position.y = err_y + self.uav_vision_pose.pose.position.y
        self.uav_set_position.pose.position.z = 0.8 #self.height_const

        if - np.pi/4. > err_yaw or err_yaw > np.pi/4.:
            cur_euler = tf.transformations.euler_from_quaternion(self.uav_vision_pose.pose.orientation)
            cur_euler[2] += err_yaw
            qut = tf.transformations.quaternion_from_euler(cur_euler[0], cur_euler[1], cur_euler[2])
            self.uav_set_position.pose.orientation.x = qut[0]
            self.uav_set_position.pose.orientation.y = qut[1]
            self.uav_set_position.pose.orientation.z = qut[2]
            self.uav_set_position.pose.orientation.w = qut[3]
        else:
            self.uav_set_position.pose.orientation.x = self.uav_vision_pose.pose.orientation.x
            self.uav_set_position.pose.orientation.y = self.uav_vision_pose.pose.orientation.y
            self.uav_set_position.pose.orientation.z = self.uav_vision_pose.pose.orientation.z
            self.uav_set_position.pose.orientation.w = self.uav_vision_pose.pose.orientation.w

        self.pub_uav_move.publish(self.uav_set_position)

if __name__ == '__main__':
    rospy.init_node('pbvs_control', anonymous=True)
    # pbvs_control = Pbvs()
    hybrid_control = HybridVS()

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        # hybrid_control.pbvs.tracker.get_now_tf()
        hybrid_control.pbvs.tracker.get_theta_u()

        if hybrid_control.pbvs.tracker.get_now_tf() and not hybrid_control.grasp_flag:
            if not hybrid_control.grasp_flag:

                hybrid_control.pub_joint_data()

                if hybrid_control.get_fly_target_depth() > 0.55:
                    hybrid_control.move_uav_to_target()


            # hybrid_control.hybrid_interaction_mat()
        if hybrid_control.grasp_flag:
                # rospy.loginfo("haha")
                hybrid_control.grasp()

        rate.sleep()
