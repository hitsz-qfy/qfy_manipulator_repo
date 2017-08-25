#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import track_2
import tf
from qfy_dynamixel.srv import *

from dynamixel_msgs.msg import MultiJointCommand
from dynamixel_msgs.msg import MotorStateFloatList, MotorStateFloat
from dynamixel_driver.dynamixel_const import *
from qfy_dynamixel.msg import multi_joint_point, PbvsErrorStamped
from qfy_ibvs.msg import ImagePoint2D, FourImagePointStamped
import pbvs_node
from numpy.linalg import inv
import cv2

class Ibvs(object):
    def __init__(self):
        self.tracker = track_2.Track()

        self.sub_apriltag = rospy.Subscriber('/points2D_info', FourImagePointStamped, self.apriltag_points_cb)

        self.cur_interaction_mat = np.matrix(np.zeros((8,6)))
        self.des_interaction_mat = np.matrix(np.zeros((8, 6)))
        self.mean_interaction_mat = np.matrix(np.zeros((8, 6)))
        self.original_interaction_mat = np.matrix(np.zeros((8, 6)))
        self.error_mat = np.matrix(np.zeros((8,1)))
        self.weight_mat = np.asmatrix(np.identity(14))
        self.h_vec = np.asmatrix(np.zeros((8,1)))

        self.camera_intrinsic_mat = np.asmatrix([[574.666883, 0.000000, 335.073855],
                                                 [0.000000, 574.765706, 247.464029],
                                                 [0.000000, 0.000000, 1.000000]])

        self.point_depth = [0., 0., 0., 0.]
        self.point0_obj = np.asmatrix([- 0.032/2, - 0.032/2, 0 , 1])
        self.point1_obj = np.asmatrix([0.032/2, - 0.032/2, 0, 1])
        self.point2_obj = np.asmatrix([0.032/2, 0.032/2, 0, 1])
        self.point3_obj = np.asmatrix([- 0.032/2, 0.032/2, 0, 1])


        self.des_point0 = ImagePoint2D()
        self.des_point0.pixel.x = 254.
        self.des_point0.pixel.y = 341.
        vec_point0_pixel = np.asmatrix([self.des_point0.pixel.x, self.des_point0.pixel.y, 1]).T
        self.des_point0_metric_mat = self.camera_intrinsic_mat.I * vec_point0_pixel
        self.des_point0.normalized.x = self.des_point0_metric_mat.item(0)
        self.des_point0.normalized.y = self.des_point0_metric_mat.item(1)

        self.des_point1 = ImagePoint2D()
        self.des_point1.pixel.x = 343.
        self.des_point1.pixel.y = 343.
        vec_point1_pixel = np.asmatrix([self.des_point1.pixel.x, self.des_point1.pixel.y, 1]).T
        self.des_point1_metric_mat = self.camera_intrinsic_mat.I * vec_point1_pixel
        self.des_point1.normalized.x = self.des_point1_metric_mat.item(0)
        self.des_point1.normalized.y = self.des_point1_metric_mat.item(1)

        self.des_point2 = ImagePoint2D()
        self.des_point2.pixel.x = 346.
        self.des_point2.pixel.y = 249.
        vec_point2_pixel = np.asmatrix([self.des_point2.pixel.x, self.des_point2.pixel.y, 1]).T
        self.des_point2_metric_mat = self.camera_intrinsic_mat.I * vec_point2_pixel
        self.des_point2.normalized.x = self.des_point2_metric_mat.item(0)
        self.des_point2.normalized.y = self.des_point2_metric_mat.item(1)

        self.des_point3 = ImagePoint2D()
        self.des_point3.pixel.x = 256.
        self.des_point3.pixel.y = 246.
        vec_point3_pixel = np.asmatrix([self.des_point3.pixel.x, self.des_point3.pixel.y, 1]).T
        self.des_point3_metric_mat = self.camera_intrinsic_mat.I * vec_point3_pixel
        self.des_point3.normalized.x = self.des_point3_metric_mat.item(0)
        self.des_point3.normalized.y = self.des_point3_metric_mat.item(1)

        self.des_depth = 0.2

        self.get_tf = False

        self.safe_roi = 0.4 ###0 to 0.5
        self.safe_range = {'xs_plus': 210., 'xs_minus': 400., 'ys_plus': 130., 'ys_minus': 310.}
        self.image_border = {'x_up':  640., 'x_down': 0., 'y_up': 480., 'y_down': 0.}

        self.cur_points_metric_list = []
        self.cur_points_pixel_list = []
        self.des_points_metric_list = [self.des_point0.normalized.x, self.des_point0.normalized.y,
                                       self.des_point1.normalized.x, self.des_point1.normalized.y,
                                       self.des_point2.normalized.x, self.des_point2.normalized.y,
                                       self.des_point3.normalized.x, self.des_point3.normalized.y]

    def calcu_error(self, points):
        # rospy.loginfo(
        #     "des point0 normalized : %.4f, %.4f" % (self.des_point0.normalized.x, self.des_point0.normalized.y))
        # rospy.loginfo(
        #     "des point1 normalized : %.4f, %.4f" % (self.des_point1.normalized.x, self.des_point1.normalized.y))
        # rospy.loginfo(
        #     "des point2 normalized : %.4f, %.4f" % (self.des_point2.normalized.x, self.des_point2.normalized.y))
        # rospy.loginfo(
        #     "des point3 normalized : %.4f, %.4f" % (self.des_point3.normalized.x, self.des_point3.normalized.y))
        self.error_mat.itemset(0, (points.p0.normalized.x - self.des_point0.normalized.x))
        self.error_mat.itemset(1, (points.p0.normalized.y - self.des_point0.normalized.y))
        self.error_mat.itemset(2, (points.p1.normalized.x - self.des_point1.normalized.x))
        self.error_mat.itemset(3, (points.p1.normalized.y - self.des_point1.normalized.y))
        self.error_mat.itemset(4, (points.p2.normalized.x - self.des_point2.normalized.x))
        self.error_mat.itemset(5, (points.p2.normalized.y - self.des_point2.normalized.y))
        self.error_mat.itemset(6, (points.p3.normalized.x - self.des_point3.normalized.x))
        self.error_mat.itemset(7, (points.p3.normalized.y - self.des_point3.normalized.y))
        # rospy.logwarn("ibvs error_mat: %s"%self.error_mat)
        return self.error_mat

    ########### OK ###############
    def calcu_interaction_mat(self):
        if len(self.cur_points_metric_list):
            # interaction_mat_tmp = np.matrix(np.zeros((8,6)))
            # interaction_mat_tmp.itemset((0,0), -1. /self.tracker.trans[2])
            # interaction_mat_tmp.itemset((0,1), 0.)
            # interaction_mat_tmp.itemset((0,2), self.cur_points_metric_list[0] / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((0,3), self.cur_points_metric_list[0] * self.cur_points_metric_list[0 + 1])
            # interaction_mat_tmp.itemset((0,4), - (1 + self.cur_points_metric_list[0] * self.cur_points_metric_list[0]))
            # interaction_mat_tmp.itemset((0,5), self.cur_points_metric_list[0 + 1])
            #
            # interaction_mat_tmp.itemset((1, 0), 0.)
            # interaction_mat_tmp.itemset((1, 1), -1. / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((1, 2), self.cur_points_metric_list[1] / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((1, 3), (1 + self.cur_points_metric_list[1] * self.cur_points_metric_list[1]))
            # interaction_mat_tmp.itemset((1, 4), - self.cur_points_metric_list[1] * self.cur_points_metric_list[1 - 1])
            # interaction_mat_tmp.itemset((1, 5), - self.cur_points_metric_list[1 - 1])
            #
            # interaction_mat_tmp.itemset((2, 0), -1. / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((2, 1), 0.)
            # interaction_mat_tmp.itemset((2, 2), self.cur_points_metric_list[2] / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((2, 3), self.cur_points_metric_list[2] * self.cur_points_metric_list[2 + 1])
            # interaction_mat_tmp.itemset((2, 4),
            #                              - (1 + self.cur_points_metric_list[2] * self.cur_points_metric_list[2]))
            # interaction_mat_tmp.itemset((2, 5), self.cur_points_metric_list[2 + 1])
            #
            # interaction_mat_tmp.itemset((3, 0), 0.)
            # interaction_mat_tmp.itemset((3, 1), -1. / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((3, 2), self.cur_points_metric_list[3] / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((3, 3), (1 + self.cur_points_metric_list[3] * self.cur_points_metric_list[3]))
            # interaction_mat_tmp.itemset((3, 4), - self.cur_points_metric_list[3] * self.cur_points_metric_list[3 - 1])
            # interaction_mat_tmp.itemset((3, 5), - self.cur_points_metric_list[3 - 1])
            #
            # interaction_mat_tmp.itemset((4, 0), -1. / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((4, 1), 0.)
            # interaction_mat_tmp.itemset((4, 2), self.cur_points_metric_list[4] / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((4, 3), self.cur_points_metric_list[4] * self.cur_points_metric_list[4 + 1])
            # interaction_mat_tmp.itemset((4, 4),
            #                              - (1 + self.cur_points_metric_list[4] * self.cur_points_metric_list[4]))
            # interaction_mat_tmp.itemset((4, 5), self.cur_points_metric_list[4 + 1])
            #
            # interaction_mat_tmp.itemset((5, 0), 0.)
            # interaction_mat_tmp.itemset((5, 1), -1. / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((5, 2), self.cur_points_metric_list[5] / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((5, 3), (1 + self.cur_points_metric_list[5] * self.cur_points_metric_list[5]))
            # interaction_mat_tmp.itemset((5, 4), - self.cur_points_metric_list[5] * self.cur_points_metric_list[5 - 1])
            # interaction_mat_tmp.itemset((5, 5), - self.cur_points_metric_list[5 - 1])
            #
            # interaction_mat_tmp.itemset((6, 0), -1. / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((6, 1), 0.)
            # interaction_mat_tmp.itemset((6, 2), self.cur_points_metric_list[6] / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((6, 3), self.cur_points_metric_list[6] * self.cur_points_metric_list[6 + 1])
            # interaction_mat_tmp.itemset((6, 4),
            #                              - (1 + self.cur_points_metric_list[6] * self.cur_points_metric_list[6]))
            # interaction_mat_tmp.itemset((6, 5), self.cur_points_metric_list[6 + 1])
            #
            # interaction_mat_tmp.itemset((7, 0), 0.)
            # interaction_mat_tmp.itemset((7, 1), -1. / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((7, 2), self.cur_points_metric_list[7] / self.tracker.trans[2])
            # interaction_mat_tmp.itemset((7, 3), (1 + self.cur_points_metric_list[7] * self.cur_points_metric_list[7]))
            # interaction_mat_tmp.itemset((7, 4), - self.cur_points_metric_list[7] * self.cur_points_metric_list[7 - 1])
            # interaction_mat_tmp.itemset((7, 5), - self.cur_points_metric_list[7 - 1])
            #
            # rospy.logwarn(interaction_mat_tmp)

            ################ Version 1 ######################
            ########### L_sstar ###########

            # i = 0
            # while i < 8:
            #     if i % 2 == 0:
            #         self.interaction_mat.itemset((i, 0),
            #                                      - 1. / self.des_depth)
            #         self.interaction_mat.itemset((i, 1),
            #                                      0.)
            #         self.interaction_mat.itemset((i, 2),
            #                                      self.des_points_metric_list[i] / self.des_depth)
            #         self.interaction_mat.itemset((i, 3),
            #                                      self.des_points_metric_list[i] * self.des_points_metric_list[i + 1])
            #         self.interaction_mat.itemset((i, 4),
            #                                      - (
            #                                      1 + self.des_points_metric_list[i] * self.des_points_metric_list[i]))
            #         self.interaction_mat.itemset((i, 5),
            #                                      self.des_points_metric_list[i + 1])
            #
            #         i += 1
            #     else:
            #         self.interaction_mat.itemset((i, 0),
            #                                      0.)
            #         self.interaction_mat.itemset((i, 1),
            #                                      - 1. / self.des_depth)
            #         self.interaction_mat.itemset((i, 2),
            #                                      self.des_points_metric_list[i] / self.des_depth)
            #         self.interaction_mat.itemset((i, 3),
            #                                      (1 + self.des_points_metric_list[i] * self.des_points_metric_list[i]))
            #         self.interaction_mat.itemset((i, 4),
            #                                      - self.des_points_metric_list[i] * self.des_points_metric_list[i - 1])
            #         self.interaction_mat.itemset((i, 5),
            #                                      - self.des_points_metric_list[i - 1])
            #         # rospy.loginfo('x_%d: %.4f, y_%d: %.4f' % (
            #         # i/2, self.cur_points_metric_list[i - 1], i/2, self.cur_points_metric_list[i]))
            #         i += 1

            ########### L_s ############
            ####可能是每个点的深度不准导致的＃＃＃＃＃
            i = 0
            while i < 8:
                if i%2 == 0:
                    ##########original interaction matrix #######
                    self.original_interaction_mat.itemset((i, 0),
                                                     - 1. / self.tracker.trans[2])
                    self.original_interaction_mat.itemset((i, 1),
                                                     0.)
                    self.original_interaction_mat.itemset((i, 2),
                                                     self.cur_points_metric_list[i] / self.tracker.trans[2])
                    self.original_interaction_mat.itemset((i, 3),
                                                     self.cur_points_metric_list[i] * self.cur_points_metric_list[
                                                         i + 1])
                    self.original_interaction_mat.itemset((i, 4),
                                                     - (
                                                     1 + self.cur_points_metric_list[i] * self.cur_points_metric_list[
                                                         i]))
                    self.original_interaction_mat.itemset((i, 5),
                                                     self.cur_points_metric_list[i + 1])

                    #########current interaction matrix ############
                    self.cur_interaction_mat.itemset((i, 0),
                                                 - 1. / self.point_depth[i/2])
                    self.cur_interaction_mat.itemset((i, 1),
                                                 0.)
                    self.cur_interaction_mat.itemset((i, 2),
                                                 self.cur_points_metric_list[i] / self.point_depth[i/2])
                    self.cur_interaction_mat.itemset((i, 3),
                                                 self.cur_points_metric_list[i] * self.cur_points_metric_list[i + 1])
                    self.cur_interaction_mat.itemset((i, 4),
                                                 - (1 + self.cur_points_metric_list[i] * self.cur_points_metric_list[i]))
                    self.cur_interaction_mat.itemset((i, 5),
                                                 self.cur_points_metric_list[i + 1])

                    #######desired interaction matrix ##############
                    self.des_interaction_mat.itemset((i, 0),
                                                     - 1. / self.des_depth)
                    self.des_interaction_mat.itemset((i, 1),
                                                     0.)
                    self.des_interaction_mat.itemset((i, 2),
                                                     self.des_points_metric_list[i] / self.des_depth)
                    self.des_interaction_mat.itemset((i, 3),
                                                     self.des_points_metric_list[i] * self.des_points_metric_list[
                                                         i + 1])
                    self.des_interaction_mat.itemset((i, 4),
                                                     - (
                                                     1 + self.des_points_metric_list[i] * self.des_points_metric_list[
                                                         i]))
                    self.des_interaction_mat.itemset((i, 5),
                                                     self.des_points_metric_list[i + 1])

                    ########mean interaction matrix ###########
                    self.mean_interaction_mat.itemset((i, 0),
                                                     (self.cur_interaction_mat.item(i, 0) +
                                                      self.des_interaction_mat.item(i, 0)))
                    self.mean_interaction_mat.itemset((i, 1),
                                                     (self.cur_interaction_mat.item(i, 1) +
                                                              self.des_interaction_mat.item(i, 1)))
                    self.mean_interaction_mat.itemset((i, 2),
                                                     (self.cur_interaction_mat.item(i, 2) +
                                                              self.des_interaction_mat.item(i, 2)))
                    self.mean_interaction_mat.itemset((i, 3),
                                                     (self.cur_interaction_mat.item(i, 3) +
                                                              self.des_interaction_mat.item(i, 3)))
                    self.mean_interaction_mat.itemset((i, 4),
                                                     (self.cur_interaction_mat.item(i, 4) +
                                                              self.des_interaction_mat.item(i, 4)))
                    self.mean_interaction_mat.itemset((i, 5),
                                                     (self.cur_interaction_mat.item(i, 5) +
                                                      self.des_interaction_mat.item(i, 5)))


                    i += 1
                else:
                    ##########original interaction matrix #########
                    self.original_interaction_mat.itemset((i, 0),
                                                     0.)
                    self.original_interaction_mat.itemset((i, 1),
                                                     - 1. / self.tracker.trans[2])
                    self.original_interaction_mat.itemset((i, 2),
                                                     self.cur_points_metric_list[i] / self.tracker.trans[2])
                    self.original_interaction_mat.itemset((i, 3),
                                                     (1 + self.cur_points_metric_list[i] * self.cur_points_metric_list[
                                                         i]))
                    self.original_interaction_mat.itemset((i, 4),
                                                     - self.cur_points_metric_list[i] * self.cur_points_metric_list[
                                                         i - 1])
                    self.original_interaction_mat.itemset((i, 5),
                                                     - self.cur_points_metric_list[i - 1])

                    ##########current interaction matrix #########
                    self.cur_interaction_mat.itemset((i, 0),
                                                 0.)
                    self.cur_interaction_mat.itemset((i, 1),
                                                 - 1. / self.point_depth[i/2])
                    self.cur_interaction_mat.itemset((i, 2),
                                                 self.cur_points_metric_list[i] / self.point_depth[i/2])
                    self.cur_interaction_mat.itemset((i, 3),
                                                 (1 + self.cur_points_metric_list[i] * self.cur_points_metric_list[i]))
                    self.cur_interaction_mat.itemset((i, 4),
                                                 - self.cur_points_metric_list[i] * self.cur_points_metric_list[i - 1])
                    self.cur_interaction_mat.itemset((i, 5),
                                                 - self.cur_points_metric_list[i - 1])


                    ########desired interaction matrix #########
                    self.des_interaction_mat.itemset((i, 0),
                                                     0.)
                    self.des_interaction_mat.itemset((i, 1),
                                                     - 1. / self.des_depth)
                    self.des_interaction_mat.itemset((i, 2),
                                                     self.des_points_metric_list[i] / self.des_depth)
                    self.des_interaction_mat.itemset((i, 3),
                                                     (1 + self.des_points_metric_list[i] * self.des_points_metric_list[
                                                         i]))
                    self.des_interaction_mat.itemset((i, 4),
                                                     - self.des_points_metric_list[i] * self.des_points_metric_list[
                                                         i - 1])
                    self.des_interaction_mat.itemset((i, 5),
                                                     - self.des_points_metric_list[i - 1])

                    #######mean interaction matrix ##############
                    self.mean_interaction_mat.itemset((i, 0),
                                                     (self.cur_interaction_mat.item(i, 0) +
                                                              self.des_interaction_mat.item(i, 0)))
                    self.mean_interaction_mat.itemset((i, 1),
                                                     (self.cur_interaction_mat.item(i, 1) +
                                                              self.des_interaction_mat.item(i, 1)))
                    self.mean_interaction_mat.itemset((i, 2),
                                                     (self.cur_interaction_mat.item(i, 2) +
                                                              self.des_interaction_mat.item(i, 2)))
                    self.mean_interaction_mat.itemset((i, 3),
                                                     (self.cur_interaction_mat.item(i, 3) +
                                                              self.des_interaction_mat.item(i, 3)))
                    self.mean_interaction_mat.itemset((i, 4),
                                                     (self.cur_interaction_mat.item(i, 4) +
                                                              self.des_interaction_mat.item(i, 4)))
                    self.mean_interaction_mat.itemset((i, 5),
                                                     (self.cur_interaction_mat.item(i, 5) +
                                                              self.des_interaction_mat.item(i, 5)))

                    # rospy.loginfo('x_%d: %.4f, y_%d: %.4f' % (
                    # i/2, self.cur_points_metric_list[i - 1], i/2, self.cur_points_metric_list[i]))
                    i += 1
            # rospy.loginfo(self.interaction_mat)

            ####################################################3

            # rospy.loginfo(self.interaction_mat)
            # err_mat_tmp = np.matrix(np.zeros((8,1))))
        # self.linear_v.itemset(1, camera_vel.item(1))
        # self.linear_v.itemset(2, camera_vel.item(2))
        #
        # self.angle_v.itemset(0, camera_vel.item(3))
        # self.angle_v.itemset(1, camera_vel.item(4))
        # self.angle_v.itemset(2, camera_vel.item(5))
            # err_mat_tmp.itemset(0, self.cur_points_metric_list[0] - self.des_point0.normalized.x)
            # err_mat_tmp.itemset(1, self.cur_points_metric_list[1] - self.des_point0.normalized.y)
            # err_mat_tmp.itemset(2, self.cur_points_metric_list[2] - self.des_point1.normalized.x)
            # err_mat_tmp.itemset(3, self.cur_points_metric_list[3] - self.des_point1.normalized.y)
            # err_mat_tmp.itemset(4, self.cur_points_metric_list[4] - self.des_point2.normalized.x)
            # err_mat_tmp.itemset(5, self.cur_points_metric_list[5] - self.des_point2.normalized.y)
            # err_mat_tmp.itemset(6, self.cur_points_metric_list[6] - self.des_point3.normalized.x)
            # err_mat_tmp.itemset(7, self.cur_points_metric_list[7] - self.des_point3.normalized.y)
            #
            #
            # camera_vel = - inv(self.interaction_mat.T * self.interaction_mat) * self.interaction_mat.T * err_mat_tmp
            # rospy.logwarn(camera_vel)

            # return self.original_interaction_mat
            return self.cur_interaction_mat
            # return self.des_interaction_mat
            # return self.mean_interaction_mat

    def check_get_tf(self):
        if self.tracker.get_now_tf():
            return self.tracker.get_tf
        else:
            return self.tracker.get_tf

    ######## OK ###########
    def get_entries(self, list_):
        self.calcu_safe_range()
        for i in xrange(len(list_)):
            if i%2 == 0:
                if list_[i] > self.safe_range['xs_plus']:
                    self.h_vec.itemset(i, (list_[i] - self.safe_range['xs_plus']) /
                                       (self.image_border['x_up'] - list_[i]))
                elif list_[i] < self.safe_range['xs_minus']:
                    self.h_vec.itemset(i, (list_[i] - self.safe_range['xs_minus']) /
                                       (self.image_border['x_down'] - list_[i]))
                else:
                    self.h_vec.itemset(i, 0.001)
            else:
                if list_[i] > self.safe_range['ys_plus']:
                    self.h_vec.itemset(i, (list_[i] - self.safe_range['ys_plus']) /
                                       (self.image_border['y_up'] - list_[i]))
                elif list_[i] < self.safe_range['ys_minus']:
                    self.h_vec.itemset(i, (list_[i] - self.safe_range['ys_minus']) /
                                       (self.image_border['y_down'] - list_[i]))
                else:
                    self.h_vec.itemset(i, 0.001)
        # self.h_vec *= 2
        # rospy.logwarn("h_vec element: %s"%self.h_vec)

    def calcu_safe_range(self):
        self.safe_range['xs_minus'] = self.image_border['x_down'] + self.safe_roi * \
                                                                    (self.image_border['x_up'] - self.image_border['x_down'])
        self.safe_range['xs_plus']  = self.image_border['x_up'] - self.safe_roi * \
                                                                  (self.image_border['x_up'] - self.image_border['x_down'])
        self.safe_range['ys_minus'] = self.image_border['y_down'] + self.safe_roi * \
                                                                    (self.image_border['y_up'] - self.image_border['y_down'])
        self.safe_range['ys_plus']  = self.image_border['y_up'] - self.safe_roi * \
                                                                  (self.image_border['y_up'] - self.image_border['y_down'])
        # rospy.logwarn("xs_minus: %.4f, xs_plus: %.4f"%(self.safe_range['xs_minus'], self.safe_range['xs_plus']))
        return self.safe_range

    def calcu_weight_mat(self):
        if len(self.cur_points_pixel_list):
            self.get_entries(self.cur_points_pixel_list)
            self.weight_mat[np.arange(6,14), np.arange(6,14)] = self.h_vec.T.tolist()[0]
            # rospy.loginfo(self.weight_mat)
            return self.weight_mat

    def get_point_depth(self):
        self.tracker.get_theta_u()
        rotation = tf.transformations.quaternion_matrix(self.tracker.rot)
        trans = np.asmatrix(self.tracker.trans).T
        # R_T = np.concatenate([rotation, trans], axis=1)
        R_T = np.concatenate([np.concatenate([rotation[np.ix_([0,1,2],[0,1,2])], trans], axis=1), np.matrix([0,0,0,1])], axis=0)
        intrinsic_mat = np.concatenate([self.camera_intrinsic_mat, np.matrix([0,0,0]).T], axis=1)
        p0 = intrinsic_mat * R_T * self.point0_obj.T
        p0.itemset(0, p0.item(0) / p0.item(2))
        p0.itemset(1, p0.item(1) / p0.item(2))

        p1 = intrinsic_mat * R_T * self.point1_obj.T
        p1.itemset(0, p1.item(0) / p1.item(2))
        p1.itemset(1, p1.item(1) / p1.item(2))

        p2 = intrinsic_mat * R_T * self.point2_obj.T
        p2.itemset(0, p2.item(0) / p2.item(2))
        p2.itemset(1, p2.item(1) / p2.item(2))

        p3 = intrinsic_mat * R_T * self.point3_obj.T
        p3.itemset(0, p3.item(0) / p3.item(2))
        p3.itemset(1, p3.item(1) / p3.item(2))

        # rospy.loginfo("\np0: x: %.4f, y: %.4f\np1: x: %.4f, y: %.4f\np2: x: %.4f, y: %.4f\np3: x: %.4f, y: %.4f" %
        #               (p0.item(0),p0.item(1), p1.item(0), p1.item(1), p2.item(0), p2.item(1), p3.item(0), p3.item(1)))
        # rospy.logwarn("\np0: z: %.4f\np1: z: %.4f\np2: z: %.4f\np3: z: %.4f"%
        #               (p0.item(2), p1.item(2), p2.item(2), p3.item(2)))
        # inv_R_T = R_T.I
        # print inv_R_T

        # point0_c = R_T * self.point0_obj.T
        # point1_c = R_T * self.point1_obj.T
        # point2_c = R_T * self.point2_obj.T
        # point3_c = R_T * self.point3_obj.T

        # point0_c = inv_R_T * self.point0_obj.T
        # point1_c = inv_R_T * self.point1_obj.T
        # point2_c = inv_R_T * self.point2_obj.T
        # point3_c = inv_R_T * self.point3_obj.T
        self.point_depth = [p0.item(2), p1.item(2), p2.item(2), p3.item(2)]
        # rospy.logwarn("\npoint 0: %.4f\npoint 1: %.4f\npoint 2: %.4f\npoint 3: %.4f"%
        #               (self.point_depth[0], self.point_depth[1], self.point_depth[2], self.point_depth[3]))

    def apriltag_points_cb(self,msg):
        self.calcu_error(msg)
        # self.tracker.get_theta_u()
        # rotation , jac = cv2.Rodrigues(self.tracker.theta_u)
        # trans = np.asmatrix(self.tracker.trans).T
        # R_T = np.concatenate([rotation, trans], axis=1)
        # point0_c = R_T * self.point0_obj.T
        # point1_c = R_T * self.point1_obj.T
        # point2_c = R_T * self.point2_obj.T
        # point3_c = R_T * self.point3_obj.T
        # self.point_depth = [point0_c.item(2), point1_c.item(2), point2_c.item(2), point3_c.item(2)]
        # print self.point_depth
        # print R_T
        # print(tf.transformations.euler_from_matrix(rotation))
        self.get_point_depth()

        if self.check_get_tf():
            self.cur_points_metric_list = [msg.p0.normalized.x, msg.p0.normalized.y,
                               msg.p1.normalized.x, msg.p1.normalized.y,
                               msg.p2.normalized.x, msg.p2.normalized.y,
                               msg.p3.normalized.x, msg.p3.normalized.y]
            self.cur_points_pixel_list = [msg.p0.pixel.x, msg.p0.pixel.y,
                                     msg.p1.pixel.x, msg.p1.pixel.y,
                                     msg.p2.pixel.x, msg.p2.pixel.y,
                                     msg.p3.pixel.x, msg.p3.pixel.y]

            self.calcu_interaction_mat()
            self.calcu_weight_mat()
