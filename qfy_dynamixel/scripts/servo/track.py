#!/usr/bin/env python
# coding=utf-8

from kinematic import *
import numpy as np
import tf
import rospy
import math
from qfy_dynamixel.msg import multi_joint_point
from control_msgs.msg import FollowJointTrajectoryActionResult

# Oec = [-0.09683, 0.2514, 0.1072]#real
# Oec = [-96.83, 0, 107.2]#virtual
#camera wrt vrpn
Oec = [-0.05, 0.00, 0.05]
Rec_tmp = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(-np.pi/2, 0.0, -np.pi/2))#(-1.603135, 0.14845945, -1.5597623)
Rec = np.vstack(([tmp[:3] for tmp in Rec_tmp[:3]]))
Tec_array = np.row_stack((np.column_stack((Rec,Oec)),[0,0,0,1]))
Tec_mat = np.mat(Tec_array)

Ooc = [0.0, 0.0, 0.0]
Roc_tmp = tf.transformations.rotation_matrix(np.pi/2,(0,0,1))
Roc = np.vstack(([tmp[:3] for tmp in Roc_tmp[:3]]))
Toc_array = np.row_stack((np.column_stack((Roc,Ooc)),[0,0,0,1]))
Toc_mat = np.mat(Toc_array)

Rat_mat = tf.transformations.rotation_matrix(-np.pi/2,(0,0,1))


Rce = Rec.T
Oce = np.asarray(-np.mat(Rec.T).dot(Oec)).tolist()[0]
Tce_array = np.row_stack((np.column_stack((Rce,Oce)),[0,0,0,1]))
Tce_mat = np.mat(Tce_array)

O_const = [0,0,0]
R_const_emp = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, -np.pi/2, 0))#(0, -np.pi/2, 0)
R_const = np.vstack(([tmp[:3] for tmp in R_const_emp[:3]]))
T_const_array = np.row_stack((np.column_stack((R_const,O_const)),[0,0,0,1]))
T_const_mat = np.mat(T_const_array)

Otc = [0,0,0]
Rtc_tmp = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(np.pi, 0, 0))
Rtc = np.vstack(([tmp[:3] for tmp in Rtc_tmp[:3]]))
Ttc_array = np.row_stack((np.column_stack((Rtc,Otc)),[0,0,0,1]))
Ttc_mat = np.mat(Ttc_array)


Rc_ereal_tmp = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, 0, np.pi/2))#(0, 0, -np.pi/2)
Rc_ereal = np.vstack(([tmp[:3] for tmp in Rc_ereal_tmp[:3]]))
Tc_ereal_array = np.row_stack((np.column_stack((Rc_ereal,O_const)),[0,0,0,1]))
Tc_ereal_mat = np.mat(Tc_ereal_array)




def invekine(n,a,P,a1=0.007,a2=0.280,d4=0.10387,d6=0.095):
    [nx,ny,nz]=n
    [ax,ay,az]=a
    [Px,Py,Pz]=P

    Px_ = Px - d6 * ax
    Py_ = Py - d6 * ay
    Pz_ = Pz - d6 * az

    ox = ay * nz - az * ny
    oy = -ax * nz + az * nx
    oz = ax * ny - ay * nx

    if Px_>0:
        theta1 = math.atan2(Py_, Px_)
    else:
        theta1 = math.atan2(-Py_, -Px_)

    # rospy.loginfo("theta1: %f"%(theta1))

    if theta1<-np.pi/2 or theta1>np.pi/2:
        rospy.logerr("ERROR theta1: %f"%(theta1+np.pi/2))
        return False
    # else:
    #     print("RIGHT theta1: %f"%(theta1+np.pi/2))
    # theta1 = math.atan2(Py_, Px_)
    # print theta1
    cv = ((Px_ + a1 * math.cos(theta1)) ** 2 + (Py_ + a1 * math.sin(theta1)) ** 2 + Pz_ ** 2 - a2 ** 2 - d4 ** 2) / (2 * a2 * d4)
    if cv**2 >=1:
        rospy.logerr("cv over 1: %f"%cv)
        return False

    # else:
    #     print("RIGHT cv: %f"%cv)


    theta3 = math.atan2(math.sqrt(1 - cv ** 2), cv)
    if theta3<0 or theta3 >np.pi:
        rospy.logerr("ERROR theta3: %f"%(theta3+np.pi/2))
        return False
    # else:
    #     print("RIGHT theta3: %f"%(theta3+np.pi/2))

    theta2 = math.atan2(Pz_, math.sqrt((Px_ + a1 * math.cos(theta1)) ** 2 + (Py_ + a1 * math.sin(theta1)) ** 2)) \
             - math.atan2(d4 * math.sin(theta3), a2 + d4 * math.cos(theta3))
    if theta2<-np.pi/2 or theta2>np.pi/2:
        rospy.logerr("ERROR theta2: %f"%(theta2+np.pi))
        return False
    # else:
    #     print("RIGHT theta2: %f"%(theta2+np.pi))


    # theta2 = math.atan2(Pz_, -math.sqrt((Px_ + a1 * math.cos(theta1)) ** 2 + (Py_ + a1 * math.sin(theta1)) ** 2)) \
    #          - math.atan2(d4 * math.sin(theta3), a2 + d4 * math.cos(theta3))


    theta4 = math.atan2(math.sin(theta1) * ax - math.cos(theta1) * ay, -math.cos(theta1) * math.sin(theta2 + theta3) \
                        * ax - math.sin(theta1) * math.sin(theta2 + theta3) * ay + math.cos(theta2 + theta3) * az)
    if theta4<-np.pi or theta4>np.pi:
        rospy.logerr("ERROR theta4: %f"%theta4)
        return False
    # else:
    #     print("RIGHT theta4: %f"%theta4)

    if ax>=0:
        theta5 = -math.acos(
            math.cos(theta1) * math.cos(theta2 + theta3) * ax + math.sin(theta1) * math.cos(theta2 + theta3) * ay \
            + math.sin(theta2 + theta3) * az)
    else:
        theta5 = math.acos(
            math.cos(theta1) * math.cos(theta2 + theta3) * ax + math.sin(theta1) * math.cos(theta2 + theta3) * ay \
            + math.sin(theta2 + theta3) * az)
    # theta5 = math.acos(
    #         math.cos(theta1) * math.cos(theta2 + theta3) * ax + math.sin(theta1) * math.cos(theta2 + theta3) * ay \
    #         + math.sin(theta2 + theta3) * az)
    if theta5<-np.pi/2 or theta5>np.pi/2:
        # print("ERROR theta5: %f"%(theta5-np.pi/2))
        rospy.logerr("ERROR theta5: %f" % (theta5))
        return False
    # else:
        # print("RIGHT theta5: %f"%(theta5-np.pi/2))
        # print("RIGHT theta5: %f" % (theta5))
    #####(SOLVED)problem!!!!#######
    if theta5>=0:
        theta6 = math.atan2(
            math.cos(theta1) * math.cos(theta2 + theta3) * ox + math.sin(theta1) * math.cos(theta2 + theta3) * oy \
            + math.sin(theta2 + theta3) * oz, -math.cos(theta1) * math.cos(theta2 + theta3) * nx - math.sin(theta1) \
            * math.cos(theta2 + theta3) * ny - math.sin(theta2 + theta3) * nz)
    else:
        theta6 = math.atan2(
            -(math.cos(theta1) * math.cos(theta2 + theta3) * ox + math.sin(theta1) * math.cos(theta2 + theta3) * oy \
            + math.sin(theta2 + theta3) * oz) , -(-math.cos(theta1) * math.cos(theta2 + theta3) * nx - math.sin(theta1) \
            * math.cos(theta2 + theta3) * ny - math.sin(theta2 + theta3) * nz))
    # theta6 = math.atan2(
    #     math.cos(theta1) * math.cos(theta2 + theta3) * ox + math.sin(theta1) * math.cos(theta2 + theta3) * oy \
    #     + math.sin(theta2 + theta3) * oz , -math.cos(theta1) * math.cos(theta2 + theta3) * nx - math.sin(theta1) \
    #     * math.cos(theta2 + theta3) * ny - math.sin(theta2 + theta3) * nz)
    # if theta5>0:
    #     theta6 = math.atan2(
    #         math.cos(theta1) * math.cos(theta2 + theta3) * ox + math.sin(theta1) * math.cos(theta2 + theta3) * oy \
    #         + math.sin(theta2 + theta3) * oz , -math.cos(theta1) * math.cos(theta2 + theta3) * nx - math.sin(theta1) \
    #         * math.cos(theta2 + theta3) * ny - math.sin(theta2 + theta3) * nz)
    # else:
    #     theta6 = math.atan2(
    #         -(math.cos(theta1) * math.cos(theta2 + theta3) * ox + math.sin(theta1) * math.cos(theta2 + theta3) * oy \
    #         + math.sin(theta2 + theta3) * oz), -(-math.cos(theta1) * math.cos(theta2 + theta3) * nx - math.sin(theta1) \
    #         * math.cos(theta2 + theta3) * ny - math.sin(theta2 + theta3) * nz))

    if theta6<-np.pi or theta6>np.pi:
        rospy.logerr("theta6 wrong: %f"%theta6)
        return False
    #####problem!!!!#######
    # if (np.pi - theta4)<0.2 and (np.pi - theta6)<0.2:
    #     theta4 = theta6 = 0.0
    if -0.2<(theta4+theta6)<0.2 or -0.2<(fabs(theta4+theta6)-3.14)<0.2:
        theta4 = theta6 = 0.0

    # return [theta1+1.570796, 4.71-theta2, theta3+3.1415926, theta4, theta5, theta6]
    # return [theta1+np.pi/2, theta2+np.pi, theta3+np.pi/2, theta4, theta5-np.pi/2, theta6]
    # value = [theta1 , theta2 , theta3 , theta4, theta5 , theta6]
    # value = [theta1 + np.pi / 2, theta2 + np.pi, theta3 + np.pi / 2, theta4, theta5 - np.pi / 2, theta6]
    #value = [theta1 + np.pi / 2, theta2 + np.pi, np.pi*1.5 - theta3 , theta4, theta5, theta6] #origin

    #rospy.loginfo("origin: %s"%[theta1,theta2,theta3,theta4,theta5,theta6])
    value = [-theta1 + np.pi / 2, -theta2, theta3 - np.pi/2, theta4, theta5, theta6]
    result = [round(i,2) for i in value]
    return result
    # return [theta1+1.570796, theta2+1.570796, theta3, theta4, theta5, theta6]

class Track(object):
    def __init__(self):
        self.kine = Kinematic(0.007,0.280,0.10387,0.095)
        self.Tbe_mat = self.kine.kinematic_calcu()
        self.Tbo_mat = np.mat(np.zeros((4,4)))
        self.Te_estar = np.mat(np.zeros((4,4)))
        self.Tb_estar = np.mat(np.zeros((4,4)))
        self.Tct_mat = np.mat(np.zeros((4, 4)))
        self.Tbd_mat = np.mat(np.zeros((4, 4)))
        self.Tb_cstar_mat = np.mat(np.zeros((4,4)))
        self.Rtd = np.mat(
            tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, np.pi, -np.pi / 2)))
        # self.Rtd = np.mat(
        #         tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, np.pi/2, -np.pi / 2)))
        self.listener = tf.TransformListener()
        # self.Tt_cstar = np.mat([[0.9563 ,    0,    0.2924,  -0.04],#30
        #                        [0,         1,     0,       0.06],
        #                        [-0.2924,    0,     0.9563 ,   0.13],#80
        #                        [0,         0,     0,       1]
        #                       ])
        # self.Tt_cstar = np.mat([[0, 0,  -1,  0],#30
        #                         [0, 1,  0,  0],
        #                         [1, 0,  0,  0],#80
        #                         [0, 0,  0,  1]
        #                         ])
        self.Tt_cstar = np.mat([[1, 0,  0,  -0.04],  # 30
                                [0, 1,  0,  0],
                                [0, 0,  1,  0.04],#80
                                [0, 0,  0,  1]
                                ])

        self.goal_point = multi_joint_point()
        self.previous = multi_joint_point()
        self.previous.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        self.goal_point.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6','joint_7']
        # self.pub = rospy.Publisher('joint_goal_point', multi_joint_point, queue_size=10)
        self.pub = rospy.Publisher('joint_goal_point', multi_joint_point, queue_size=10)

        self.init_time = rospy.Time.now()
        self.sub_m_trjresult = rospy.Subscriber('m_arm_controller/follow_joint_trajectory/result',
                                              FollowJointTrajectoryActionResult, self.m_result_callback)
        self.sub_a_trjresult = rospy.Subscriber('a_arm_controller/follow_joint_trajectory/result',
                                                FollowJointTrajectoryActionResult, self.a_result_callback)
        self.m_status, self.a_status = False, False
        self.first_get_tf = False
        self.no_tf_camera = False
        self.first_trj, self.second_trj = False, False
        self.half_goal = []
        self.pub_flag = True

    def m_result_callback(self,msg):
        if msg.status.status == 3:
            self.m_status = True

    def a_result_callback(self,msg):
        if msg.status.status == 3:
            self.a_status = True


    def kine_calcu(self):
        # self.kine=Kinematic(0.007,0.246,0.110,0.111)
        self.Tbe_mat = self.kine.kinematic_calcu()

    def get_current_Tc_cstar(self):
        try:
            # self.listener.waitForTransform('/camera','/target1',rospy.Time(0),rospy.Duration(10))
            (self.trans,self.rot) = self.listener.lookupTransform('/camera', '/target1', rospy.Time(0))
            # self.trans_pro = [i*1000 for i in self.trans]
            # print self.trans
            # print tf.transformations.euler_from_quaternion(self.rot)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr_throttle(60,"some tf exception happened")
            self.no_tf_camera = False
            return False

    def desire_trans(self):
        #get desired Tb_estar
        self.rotation = tf.transformations.quaternion_matrix(self.rot)
        tmp = [t[:3] for t in self.rotation[:3]]
        self.rotation = np.vstack((tmp))
        # self.Te_estar = Transform2Homo(self.rot,self.trans).trans2homo() * self.Tt_cstar

        #origin
        Tct_array = np.row_stack(
            (np.column_stack((self.rotation, self.trans)), [0, 0, 0, 1]))

        # Tct_array = np.row_stack(
        #     (np.column_stack((self.rotation, self.trans_pro)), [0, 0, 0, 1]))

        self.Tct_mat = np.mat(Tct_array)

        self.Te_t = Tec_mat * self.Tct_mat

        # self.Te_estar = np.row_stack(
        #     (np.column_stack((self.rotation, self.trans)), [0, 0, 0, 1])) * self.Tt_cstar

        # self.Te_estar = Tce*np.row_stack((np.column_stack((self.rotation,self.trans)),[0,0,0,1]))*self.Tt_cstar*Tec

        # print self.Tbe*self.Te_estar
        # return self.Tbe*self.Te_estar
        # return self.Tbe_mat * T_const_mat *self.Te_t*Ttc_mat*Tc_ereal_mat

        # return self.Tbd_mat*self.Tt_cstar
        # return self.Tbd_mat
        # print self.Tb_cstar_mat
        return self.Tb_cstar_mat
        # return self.Tbo_mat * Toc_mat * self.Tct_mat * self.Rtd * self.Tt_cstar

    def tf_pub(self):
        # print self.Tbe_mat
        # rotation_mat = np.mat(
        #     tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, 0, np.pi)))

        rotation_mat = tf.transformations.rotation_matrix(np.pi, (0,0,1))
        # rotation_mat = tf.transformations.rotation_matrix(np.pi, (1, 0, 0))

        self.Tbo_mat = self.Tbe_mat
        self.Tbe_mat = self.Tbe_mat * rotation_mat
        # self.Tbe_mat = self.Tbe_mat*rotation_mat   #const transformation
        # Tev = np.mat(tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0, -np.pi/2, 0)))#const transformation
        Tev = tf.transformations.rotation_matrix(-np.pi/2,(0,1,0))
        Tbv_mat = self.Tbe_mat*Tev

        Tbc_mat = Tbv_mat*Tec_mat

        self.Tbd_mat = Tbc_mat*self.Tct_mat*self.Rtd
        self.Tb_cstar_mat = self.Tbd_mat*self.Tt_cstar

        Tbo_array = np.array(self.Tbo_mat)
        Tbe_array = np.array(self.Tbe_mat)
        Tbv_array = np.array(Tbv_mat)
        Tbc_array = np.array(Tbc_mat)
        Tbd_array = np.array(self.Tbd_mat)
        Tbcstar_array = np.array(self.Tb_cstar_mat)

        tmp_bo = [t[:3] for t in Tbo_array[:3]]
        Tbo_rotation = np.vstack((tmp_bo))
        Mbo = np.identity(4)
        Mbo[:3, :3] = Tbo_rotation

        tmp_be = [t[:3] for t in Tbe_array[:3]]
        Tbe_rotation = np.vstack((tmp_be))
        M = np.identity(4)
        M[:3,:3] = Tbe_rotation

        tmp_bv = [t1[:3] for t1 in Tbv_array[:3]]
        Tbv_rotation = np.vstack((tmp_bv))
        Mbv = np.identity(4)
        Mbv[:3,:3] = Tbv_rotation

        tmp_bc = [t2[:3] for t2 in Tbc_array[:3]]
        Tbc_rotation = np.vstack((tmp_bc))
        Mbc = np.identity(4)
        Mbc[:3,:3] = Tbc_rotation

        tmp_bd = [t3[:3] for t3 in Tbd_array[:3]]
        Tbd_rotation = np.vstack((tmp_bd))
        Mbd = np.identity(4)
        Mbd[:3, :3] = Tbd_rotation

        tmp_bcstar = [t4[:3] for t4 in Tbcstar_array[:3]]
        Tbcstar_rotation = np.vstack((tmp_bcstar))
        Mbcstar = np.identity(4)
        Mbcstar[:3, :3] = Tbcstar_rotation


        # print Tbe_rotation

        br = tf.TransformBroadcaster()
        br.sendTransform(
            (Tbo_array[0, 3], Tbo_array[1, 3], Tbo_array[2, 3]),

            tf.transformations.quaternion_from_matrix(Mbo),
            rospy.Time.now(),
            "origin_endeffector",
            "base_link"
        )

        br.sendTransform(
            (Tbe_array[0,3],Tbe_array[1,3],Tbe_array[2,3]),
            tf.transformations.quaternion_from_matrix(M),
            rospy.Time.now(),
            "end_effector",
            "base_link"
        )
        br.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_matrix(Mbv),
            rospy.Time.now(),
            "vrpn_end_effector",
            "end_effector"
        )
        br.sendTransform(
            (Tbc_array[0,3],Tbc_array[1,3],Tbc_array[2,3]),
            tf.transformations.quaternion_from_matrix(Mbc),
            rospy.Time.now(),
            "camera",
            "base_link"
        )

        br.sendTransform(
            (Tbd_array[0, 3], Tbd_array[1, 3], Tbd_array[2, 3]),
            tf.transformations.quaternion_from_matrix(Mbd),
            rospy.Time.now(),
            "desired_pose",
            "base_link"
        )

        br.sendTransform(
            (Tbcstar_array[0, 3], Tbcstar_array[1, 3], Tbcstar_array[2, 3]),
            tf.transformations.quaternion_from_matrix(Mbcstar),
            rospy.Time.now(),
            "desired_pose_distance",
            "base_link"
        )

    def invekine(self):

        #return inverse kinematic solution + end-effector state

        self.Tbe_array = np.asarray(self.Tbe_mat)
        self.Tb_t = np.asarray(self.desire_trans())


        n = [t_n[0] for t_n in self.Tb_t[:3]]
        a = [t_a[2] for t_a in self.Tb_t[:3]]
        P = [t_P[3] for t_P in self.Tb_t[:3]]

        if invekine(n=n,a=a,P=P):
            # self.goal_point.header.stamp = rospy.Time.now()
            # self.goal_point.data = invekine(n=n,a=a,P=P) + [self.kine.joint_value[6]]
            # print("inverse kinematic: %s"%self.goal_point.data)
            rospy.loginfo('Valid invekine')
            # print invekine(n=n,a=a,P=P) + [self.kine.joint_value[6]]
            #rospy.loginfo(invekine(n=n,a=a,P=P) + [self.kine.joint_value[6]])
            return invekine(n=n,a=a,P=P) + [self.kine.joint_value[6]]
        else:
            rospy.logerr('No invekine')
            return False

    def diff_data(self):
        if sum([(self.previous.data[i]-self.goal_point.data[i])**2 for i in range(6)]) >0.05:
            self.goal_point.header.stamp = rospy.Time.now()
            self.goal_point.data = self.previous.data

    def get_cur_joint(self):
        return self.kine.cur_joint()

    def publish_goal(self):
        #version 1 : grasp one trajectory
        # t = rospy.Time.now()
        #
        # if self.get_current_Tc_cstar():
        #     if not self.first_get_tf:
        #         self.first_get_tf = True
        #         self.init_time = rospy.Time.now()
        #     else:
        #         error_t = t - self.init_time
        #         if error_t<=rospy.Duration(2):
        #             if self.invekine():
        #                 self.previous.header.stamp = rospy.Time.now()
        #                 self.previous.data = self.invekine()
        #                 self.goal_point.data = self.invekine()
        #         elif error_t<=rospy.Duration(7):
        #         # else:
        #         #     cur_joint = self.get_cur_joint()
        #         #     rospy.loginfo("current joint value: %s"%cur_joint)
        #         #     self.goal_point.data = self.invekine()
        #         #     delta_joint = list((self.goal_point.data[i]-cur_joint[i])/3.*2. for i in xrange(6))
        #         #     rospy.logwarn("delta joint value: %s"%delta_joint)
        #         #     self.goal_point.data = list(delta_joint[i]+cur_joint[i] for i in xrange(6))
        #         #     rospy.loginfo("goal point value: %s"%self.goal_point.data)
        #             self.diff_data()
        #             self.pub.publish(self.goal_point)
        #         elif error_t <= rospy.Duration(11):
        #             self.goal_point.data[6] = -0.6
        #             self.pub.publish(self.goal_point)
        #         elif error_t <= rospy.Duration(17):
        #             self.goal_point.data = [1.57, 1.5, 1.2, 0.0, 0.4, 0.0, -0.6]
        #             self.pub.publish(self.goal_point)
        #         ####test######
        #         # if self.invekine():
        #         #     self.goal_point.data = self.invekine()
        #         #     self.pub.publish(self.goal_point)
        # else:
        #     pass

        #version 2 : grasp two trajectory
        t_now = rospy.Time.now()
        if self.pub_flag:
            if not self.get_current_Tc_cstar():
                self.goal_point.header.stamp = t_now
                self.goal_point.data = [1.57, 1.55, 1.2, 0.0, 0.5, 0.0, -0.6]
                self.pub.publish(self.goal_point)
                self.pub_flag = False
            else:
                if not self.first_trj and not self.second_trj:
                    if not self.first_get_tf:
                        self.first_get_tf = True
                        self.init_time = rospy.Time.now()
                    else:
                        error_t = t_now - self.init_time
                        if error_t <= rospy.Duration(1):
                            if self.invekine():
                                self.previous.header.stamp = rospy.Time.now()
                                self.previous.data = self.invekine()
                                self.goal_point.data = self.invekine()
                                self.half_goal = list(i/2. for i in self.goal_point.data[:6]) + self.goal_point.data[6]
                        elif error_t <= rospy.Duration(7):
                            self.diff_data()
                            self.pub.publish(self.half_goal)
                            self.first_trj = True
                            self.first_get_tf = False

                if self.first_get_tf and not self.second_trj:
                    if not self.first_get_tf:
                        self.first_get_tf = True
                        self.init_time = rospy.Time.now()
                    else:
                        error_t = t_now - self.init_time
                        if error_t <= rospy.Duration(1):
                            if self.invekine():
                                self.previous.header.stamp = rospy.Time.now()
                                self.previous.data = self.invekine()
                                self.goal_point.data = self.invekine()
                        elif error_t <= rospy.Duration(7):
                            self.diff_data()
                            self.pub.publish(self.goal_point)
                            self.first_trj = True
                            self.first_get_tf = False
                            self.second_trj = True

                if self.first_get_tf and self.second_trj:
                    self.init_time = rospy.Time.now()
                    error_t = t_now - self.init_time
                    if error_t <= rospy.Duration(4):
                        self.goal_point.data[6] = -0.6
                        self.pub.publish(self.goal_point)
                    elif error_t <= rospy.Duration(9):
                        self.goal_point.data = [1.57, 1.55, 1.2, 0.0, 0.5, 0.0, -0.6]
                        self.pub.publish(self.goal_point)
                else:
                    self.pub_flag = False
        else:
            pass







